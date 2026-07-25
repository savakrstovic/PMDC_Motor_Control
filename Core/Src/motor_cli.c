#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "motor_cli.h"
#include "main.h"
#include "usart.h"
#include "motor_driver.h"
#include "speed_control.h"

/* ---------------------------------------------------------------------------
 * Interaction with the 1kHz control loop
 *
 * Interrupt priorities in this build (NVIC_PRIORITYGROUP_4 -> 4 preempt bits,
 * no subpriority):
 *
 *   SysTick   0   HAL tick (TICK_INT_PRIORITY in stm32g4xx_hal_conf.h)
 *   TIM6      1   SpeedControl_Task -- the control loop
 *   LPUART1   3   this CLI
 *
 * LPUART1 sits strictly below TIM6, so a character arriving mid-tick waits
 * for the tick to finish; the tick never waits for the CLI. The ISR itself
 * only moves one byte between the peripheral and a ring buffer -- no parsing,
 * no formatting, no HAL calls, no loops. Everything expensive (and everything
 * that can block, including MotorDriver_ClearFault's HAL_Delay) happens in
 * MotorCli_Task at thread level, where the tick preempts it freely.
 *
 * Neither direction ever blocks: a full TX ring drops output rather than
 * spinning, and a full RX ring drops input rather than stalling the ISR.
 *
 * This drives the LPUART1 registers directly rather than going through the
 * HAL_UART_Transmit/Receive_IT state machine, which cannot express a
 * free-running RX or a continuous TX stream without dropping bytes between
 * completion callback and re-arm. hlpuart1 is still used for MX init and
 * clock/GPIO setup -- but do not mix HAL_UART_Transmit() with this module.
 * ------------------------------------------------------------------------- */

#define CLI_UART         LPUART1  /* must be hlpuart1.Instance */
#define CLI_IRQn         LPUART1_IRQn
#define CLI_IRQ_PRIORITY 3U

/* Power-of-two so the wrap is a mask. */
#define RX_RING_SIZE 128U
#define TX_RING_SIZE 512U
#define RX_RING_MASK (RX_RING_SIZE - 1U)
#define TX_RING_MASK (TX_RING_SIZE - 1U)

#define CLI_LINE_MAX  48U
#define CLI_PROMPT    "> "
#define CLI_EOL       "\r\n"

#define STREAM_PERIOD_MIN_MS 50U
#define STREAM_PERIOD_MAX_MS 10000U

/* The tach's ±2048 codes at ~1.8 rpm/code cap what the loop can actually
 * measure; a setpoint past that could never be closed on. */
#define SETPOINT_LIMIT_RPM 3600.0f

/* Single-producer/single-consumer rings: for each pair, one index is written
 * only by the ISR and the other only by the task, so no locking is needed. */
static volatile uint8_t  s_rx_buf[RX_RING_SIZE];
static volatile uint16_t s_rx_head; /* ISR writes */
static volatile uint16_t s_rx_tail; /* task writes */

static volatile uint8_t  s_tx_buf[TX_RING_SIZE];
static volatile uint16_t s_tx_head; /* task writes */
static volatile uint16_t s_tx_tail; /* ISR writes */

static char     s_line[CLI_LINE_MAX];
static uint16_t s_line_len;
static bool     s_line_overflow;
static bool     s_last_was_cr;

static uint32_t s_stream_period_ms;
static uint32_t s_stream_last_ms;

/* --------------------------------------------------------------- output --- */

static void TxKick(void)
{
  /* Read-modify-write on CR1 that the ISR also touches. */
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  CLI_UART->CR1 |= USART_CR1_TXEIE_TXFNFIE;
  __set_PRIMASK(primask);
}

static void CliWrite(const char *data, size_t len)
{
  for (size_t i = 0U; i < len; i++)
  {
    uint16_t head = s_tx_head;
    uint16_t next = (uint16_t)((head + 1U) & TX_RING_MASK);

    if (next == s_tx_tail)
    {
      /* Ring full. Drop the remainder -- a truncated status line beats
       * stalling the main loop waiting on the wire. */
      break;
    }

    s_tx_buf[head] = (uint8_t)data[i];
    __DMB(); /* byte visible before the index that publishes it */
    s_tx_head = next;
  }

  TxKick();
}

static void CliPuts(const char *s)
{
  CliWrite(s, strlen(s));
}

/* ------------------------------------------------------------ formatting --- */

/* Small append-with-truncation builder. Avoids printf entirely: no %f (which
 * needs -u _printf_float), no malloc, no reentrancy questions. */
typedef struct
{
  char  *buf;
  size_t cap;
  size_t len;
} CliBuf;

static void BufChar(CliBuf *b, char c)
{
  if (b->len + 1U < b->cap)
  {
    b->buf[b->len++] = c;
    b->buf[b->len] = '\0';
  }
}

static void BufStr(CliBuf *b, const char *s)
{
  while (*s != '\0')
  {
    BufChar(b, *s++);
  }
}

/* Renders value/10 with exactly one decimal, e.g. -4133 -> "-413.3". */
static void Fixed1ToStr(char *dst, size_t cap, int32_t tenths)
{
  CliBuf b = { dst, cap, 0U };
  char digits[12];
  uint8_t n = 0U;

  bool negative = (tenths < 0);
  /* Negate in uint32 so INT32_MIN cannot overflow. */
  uint32_t magnitude = negative ? (uint32_t)(-(int64_t)tenths) : (uint32_t)tenths;

  uint32_t whole = magnitude / 10U;
  uint32_t frac = magnitude % 10U;

  dst[0] = '\0';

  do
  {
    digits[n++] = (char)('0' + (whole % 10U));
    whole /= 10U;
  } while ((whole != 0U) && (n < sizeof(digits)));

  if (negative)
  {
    BufChar(&b, '-');
  }
  while (n > 0U)
  {
    BufChar(&b, digits[--n]);
  }
  BufChar(&b, '.');
  BufChar(&b, (char)('0' + frac));
}

static void BufPadded(CliBuf *b, const char *s, size_t width)
{
  size_t n = strlen(s);
  while (n < width)
  {
    BufChar(b, ' ');
    n++;
  }
  BufStr(b, s);
}

static int32_t RpmToTenths(float rpm)
{
  /* Round half away from zero. Range here is a few thousand rpm, so no
   * int32 overflow concerns. */
  return (int32_t)((rpm * 10.0f) + ((rpm >= 0.0f) ? 0.5f : -0.5f));
}

/* ---------------------------------------------------------------- status --- */

static void PrintStatusLine(void)
{
  SpeedControl_Telemetry telemetry;
  char line[96];
  char number[16];
  CliBuf b = { line, sizeof(line), 0U };

  SpeedControl_GetTelemetry(&telemetry);
  line[0] = '\0';

  BufStr(&b, "sp=");
  Fixed1ToStr(number, sizeof(number), RpmToTenths(telemetry.setpoint_rpm));
  BufPadded(&b, number, 8U);

  BufStr(&b, " rpm  meas=");
  Fixed1ToStr(number, sizeof(number), RpmToTenths(telemetry.measured_rpm));
  BufPadded(&b, number, 8U);

  /* duty is permille of full scale, so permille == tenths of a percent. */
  BufStr(&b, " rpm  duty=");
  Fixed1ToStr(number, sizeof(number), (int32_t)telemetry.output_duty_permille);
  BufPadded(&b, number, 7U);

  BufStr(&b, " %  fault=");
  BufStr(&b, MotorDriver_IsFaulted() ? "YES" : "no");
  BufStr(&b, CLI_EOL);

  CliPuts(line);
}

/* Status printed while the user may be mid-line: break the line, print, then
 * restore the prompt and whatever had been typed so far. */
static void PrintStatusAsync(void)
{
  CliPuts(CLI_EOL);
  PrintStatusLine();
  CliPuts(CLI_PROMPT);
  if (s_line_len > 0U)
  {
    CliWrite(s_line, s_line_len);
  }
}

/* --------------------------------------------------------------- parsing --- */

static bool TokenIs(const char *token, size_t len, const char *name)
{
  return (strlen(name) == len) && (strncmp(token, name, len) == 0);
}

/* Optional sign, decimal digits, optional single '.' fraction. No exponent,
 * no trailing junk -- "500", "-500", "+12.5" ok; "5e2", "500rpm" rejected. */
static bool ParseFloat(const char *s, float *out)
{
  bool negative = false;
  bool any_digit = false;
  float value = 0.0f;

  if ((*s == '+') || (*s == '-'))
  {
    negative = (*s == '-');
    s++;
  }

  while ((*s >= '0') && (*s <= '9'))
  {
    value = (value * 10.0f) + (float)(*s - '0');
    any_digit = true;
    s++;
  }

  if (*s == '.')
  {
    float scale = 0.1f;
    s++;
    while ((*s >= '0') && (*s <= '9'))
    {
      value += (float)(*s - '0') * scale;
      scale *= 0.1f;
      any_digit = true;
      s++;
    }
  }

  if (!any_digit || (*s != '\0'))
  {
    return false;
  }

  *out = negative ? -value : value;
  return true;
}

static bool ParseUint(const char *s, uint32_t *out)
{
  uint32_t value = 0U;
  bool any_digit = false;

  while ((*s >= '0') && (*s <= '9'))
  {
    if (value > ((0xFFFFFFFFU - (uint32_t)(*s - '0')) / 10U))
    {
      return false; /* overflow */
    }
    value = (value * 10U) + (uint32_t)(*s - '0');
    any_digit = true;
    s++;
  }

  if (!any_digit || (*s != '\0'))
  {
    return false;
  }

  *out = value;
  return true;
}

/* -------------------------------------------------------------- commands --- */

static void CmdHelp(void)
{
  CliPuts("commands:" CLI_EOL
          "  set <rpm>    target speed, signed (e.g. set 500 / set -500)" CLI_EOL
          "  clear        clear the hardware fault latch" CLI_EOL
          "  status       print setpoint / measured / duty / fault" CLI_EOL
          "  stream <ms>  repeat status every <ms> (50-10000)" CLI_EOL
          "  stream off   stop repeating" CLI_EOL
          "  help         this list" CLI_EOL);
}

static void CmdSet(const char *arg)
{
  float rpm;

  if ((arg == NULL) || !ParseFloat(arg, &rpm))
  {
    CliPuts("err: usage 'set <rpm>', e.g. set -500" CLI_EOL);
    return;
  }

  if ((rpm > SETPOINT_LIMIT_RPM) || (rpm < -SETPOINT_LIMIT_RPM))
  {
    CliPuts("err: setpoint beyond tach range (+/-3600 rpm)" CLI_EOL);
    return;
  }

  SpeedControl_SetSetpointRpm(rpm);

  {
    char line[40];
    char number[16];
    CliBuf b = { line, sizeof(line), 0U };
    line[0] = '\0';

    Fixed1ToStr(number, sizeof(number), RpmToTenths(rpm));
    BufStr(&b, "setpoint = ");
    BufStr(&b, number);
    BufStr(&b, " rpm" CLI_EOL);
    CliPuts(line);
  }

  if (MotorDriver_IsFaulted())
  {
    CliPuts("note: bridge is faulted, run 'clear' before it will move" CLI_EOL);
  }
}

static void CmdClear(void)
{
  /* Blocks the main loop for ~1-2ms on HAL_Delay inside the driver. The TIM6
   * tick preempts thread mode, so the control loop keeps running throughout;
   * characters typed meanwhile land in the RX ring and are parsed after. */
  MotorDriver_ClearFault();

  if (MotorDriver_IsFaulted())
  {
    /* BKIN is level-sensitive -- still asserted means the fault is present. */
    CliPuts("fault: still asserted, condition has not cleared" CLI_EOL);
  }
  else
  {
    CliPuts("fault cleared, bridge re-enabled at zero duty" CLI_EOL);
  }
}

static void CmdStream(const char *arg)
{
  uint32_t period_ms;

  if (arg == NULL)
  {
    CliPuts("err: usage 'stream <ms>' or 'stream off'" CLI_EOL);
    return;
  }

  if (TokenIs(arg, strlen(arg), "off"))
  {
    s_stream_period_ms = 0U;
    CliPuts("stream off" CLI_EOL);
    return;
  }

  if (!ParseUint(arg, &period_ms) ||
      (period_ms < STREAM_PERIOD_MIN_MS) || (period_ms > STREAM_PERIOD_MAX_MS))
  {
    CliPuts("err: period must be 50-10000 ms, or 'off'" CLI_EOL);
    return;
  }

  s_stream_period_ms = period_ms;
  s_stream_last_ms = HAL_GetTick();
  CliPuts("stream on" CLI_EOL);
}

static void HandleLine(char *line)
{
  char *command;
  char *arg;
  size_t command_len = 0U;

  while (*line == ' ')
  {
    line++;
  }

  command = line;
  while ((line[command_len] != '\0') && (line[command_len] != ' '))
  {
    command_len++;
  }

  if (command_len == 0U)
  {
    return;
  }

  arg = &command[command_len];
  if (*arg != '\0')
  {
    *arg++ = '\0';
    while (*arg == ' ')
    {
      arg++;
    }
  }
  if (*arg == '\0')
  {
    arg = NULL;
  }
  else
  {
    /* Trim trailing spaces so "set 500  " parses. */
    char *end = arg + strlen(arg);
    while ((end > arg) && (end[-1] == ' '))
    {
      end--;
    }
    *end = '\0';
  }

  if (TokenIs(command, command_len, "set"))
  {
    CmdSet(arg);
  }
  else if (TokenIs(command, command_len, "clear"))
  {
    CmdClear();
  }
  else if (TokenIs(command, command_len, "status"))
  {
    PrintStatusLine();
  }
  else if (TokenIs(command, command_len, "stream"))
  {
    CmdStream(arg);
  }
  else if (TokenIs(command, command_len, "help") ||
           TokenIs(command, command_len, "?"))
  {
    CmdHelp();
  }
  else
  {
    CliPuts("err: unknown command, try 'help'" CLI_EOL);
  }
}

/* ------------------------------------------------------------------ task --- */

/* Returns the next received byte, or -1 when the ring is empty. */
static int RxPop(void)
{
  uint16_t tail = s_rx_tail;
  uint8_t byte;

  if (tail == s_rx_head)
  {
    return -1;
  }

  byte = s_rx_buf[tail];
  __DMB(); /* consume the byte before releasing the slot */
  s_rx_tail = (uint16_t)((tail + 1U) & RX_RING_MASK);

  return (int)byte;
}

static void HandleChar(char c)
{
  /* Terminals send CR, LF, or CRLF. Swallow the LF of a CRLF pair so one
   * Enter is one command. */
  if ((c == '\n') && s_last_was_cr)
  {
    s_last_was_cr = false;
    return;
  }
  s_last_was_cr = (c == '\r');

  if ((c == '\r') || (c == '\n'))
  {
    CliPuts(CLI_EOL);

    if (s_line_overflow)
    {
      CliPuts("err: line too long" CLI_EOL);
    }
    else if (s_line_len > 0U)
    {
      s_line[s_line_len] = '\0';
      HandleLine(s_line);
    }

    s_line_len = 0U;
    s_line_overflow = false;
    CliPuts(CLI_PROMPT);
  }
  else if ((c == '\b') || (c == 0x7F))
  {
    if (s_line_len > 0U)
    {
      s_line_len--;
      CliPuts("\b \b");
    }
  }
  else if ((c >= 0x20) && (c < 0x7F))
  {
    if (s_line_len < (CLI_LINE_MAX - 1U))
    {
      s_line[s_line_len++] = c;
      CliWrite(&c, 1U); /* echo */
    }
    else
    {
      s_line_overflow = true;
    }
  }
  /* other control characters ignored */
}

void MotorCli_Task(void)
{
  int c;

  while ((c = RxPop()) >= 0)
  {
    HandleChar((char)c);
  }

  if (s_stream_period_ms != 0U)
  {
    uint32_t now = HAL_GetTick();
    if ((now - s_stream_last_ms) >= s_stream_period_ms)
    {
      s_stream_last_ms = now;
      PrintStatusAsync();
    }
  }
}

void MotorCli_Init(void)
{
  s_rx_head = 0U;
  s_rx_tail = 0U;
  s_tx_head = 0U;
  s_tx_tail = 0U;
  s_line_len = 0U;
  s_line_overflow = false;
  s_last_was_cr = false;
  s_stream_period_ms = 0U;
  s_stream_last_ms = 0U;

  /* MX_LPUART1_UART_Init must have run, and must have configured the same
   * peripheral this module pokes directly. */
  if (hlpuart1.Instance != CLI_UART)
  {
    Error_Handler();
  }

  HAL_NVIC_SetPriority(CLI_IRQn, CLI_IRQ_PRIORITY, 0U);
  HAL_NVIC_EnableIRQ(CLI_IRQn);

  /* Drop anything latched while the line was floating during bring-up. */
  CLI_UART->ICR = USART_ICR_ORECF | USART_ICR_FECF |
                  USART_ICR_NECF | USART_ICR_PECF;
  CLI_UART->CR1 |= USART_CR1_RXNEIE_RXFNEIE;

  CliPuts(CLI_EOL "PMDC motor console -- 'help' for commands" CLI_EOL);
  CliPuts(CLI_PROMPT);
}

/* ------------------------------------------------------------------- ISR --- */

/* Not generated by CubeMX: LPUART1's global interrupt is disabled in the .ioc,
 * so the vector resolves to this instead of the weak default. If LPUART1 is
 * ever ticked on in the NVIC tab, delete the generated handler in
 * stm32g4xx_it.c or this will be a duplicate symbol. */
void LPUART1_IRQHandler(void)
{
  uint32_t isrflags = CLI_UART->ISR;
  uint32_t cr1 = CLI_UART->CR1;

  /* Overrun/framing/noise: clear and move on. ORE in particular latches and
   * would stop RXNE from ever firing again if left set. */
  if ((isrflags & (USART_ISR_ORE | USART_ISR_FE |
                   USART_ISR_NE | USART_ISR_PE)) != 0U)
  {
    CLI_UART->ICR = USART_ICR_ORECF | USART_ICR_FECF |
                    USART_ICR_NECF | USART_ICR_PECF;
  }

  if (((isrflags & USART_ISR_RXNE_RXFNE) != 0U) &&
      ((cr1 & USART_CR1_RXNEIE_RXFNEIE) != 0U))
  {
    uint8_t byte = (uint8_t)(CLI_UART->RDR & 0xFFU); /* read also clears RXNE */
    uint16_t head = s_rx_head;
    uint16_t next = (uint16_t)((head + 1U) & RX_RING_MASK);

    if (next != s_rx_tail)
    {
      s_rx_buf[head] = byte;
      __DMB();
      s_rx_head = next;
    }
    /* else: ring full, byte dropped -- the ISR never waits */
  }

  if (((isrflags & USART_ISR_TXE_TXFNF) != 0U) &&
      ((cr1 & USART_CR1_TXEIE_TXFNFIE) != 0U))
  {
    uint16_t tail = s_tx_tail;

    if (tail == s_tx_head)
    {
      CLI_UART->CR1 &= ~USART_CR1_TXEIE_TXFNFIE; /* nothing left to send */
    }
    else
    {
      CLI_UART->TDR = s_tx_buf[tail];
      s_tx_tail = (uint16_t)((tail + 1U) & TX_RING_MASK);
    }
  }
}
