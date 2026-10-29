# Timer Driver Port: G0 → G4 (context for Claude)

## How to help me
- **Guide, don't write the driver for me.** Explain, show short snippets, point to manual sections. I write the code; you review it.
- Keep it simple and straightforward. Match the existing code style (see `Peripherals/uart/uart.c`).

## Project
- MCU: **STM32G474CEU6**, bare-metal CMSIS register-level (no HAL), STM32CubeIDE.
- Clock: 16 MHz HSI, no APB prescaler → **timer clock = 16 MHz** (`BOARD_FCLK_HZ` in `Board/board_config.h`).
- Branch: `feature/timer` → PR to `main` (shared GitHub repo RakSay123).
- Layers: `Peripherals/` (register drivers) → `Devices/` (led, console) → `Board/` (`device_instances.c` holds config instances + getters, `board_init.c` calls inits) → `App/`.
- Status LED on PC6. Console on USART2 PA2/PA3 AF7.

## Task
Port the old G0 timer drivers. **Include: basic timer, PWM, encoder. Skip input capture.**
- Old source (both repos have the same timer code):
  - https://github.com/RakSay123/stm32g0-pid-experiments/tree/main/Drivers/timer
  - https://github.com/RakSay123/stm32g0-motor/tree/main/Drivers
- Order: `timer.c` → `timer_pwm.c` → `timer_encoder.c`. Test each before moving on.

## Folder layout (matters: `.cproject` is shared in git)
`Peripherals` is already a recursive source folder + include path, so no project settings change is needed:
```
Peripherals/timer/timer.c / timer.h
Peripherals/timer/timer_pwm/timer_pwm.c / .h
Peripherals/timer/timer_encoder/timer_encoder.c / .h
```
Include as `"timer/timer.h"`, like `"gpio/gpio.h"`.

## References
- RM0440 (reference manual): https://www.st.com/resource/en/reference_manual/rm0440-stm32g4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
  - RCC chapter → `RCC_APB1ENR1` (TIM2–7), `RCC_APB2ENR` (TIM1/8/15/16/17/20)
  - General-purpose timers TIM2/3/4/5 chapter → time-base unit, PWM mode, encoder interface mode, SMCR.SMS table
  - Advanced-control timers TIM1/8/20 chapter → BDTR.MOE
  - Register map at end of each timer chapter shows all bit positions
- Datasheet (pin → timer AF numbers): https://static.datasheets.com/doc/12665965-stmicroelectronics-stm32g474ceu6-ds.pdf → "Alternate function" table
- IRQ names: `IRQn_Type` enum in `stm32g474xx.h`; handler names in `Startup/startup_stm32g474ceux.s` (~lines 173–229)

## G0 → G4 changes

### timer.c
- `#include "stm32g474xx.h"` (was `stm32g070xx.h`).
- RCC: `RCC->APBENR1/2` → `RCC->APB1ENR1` / `RCC->APB2ENR`. Use CMSIS macro names (`RCC_APB1ENR1_TIM2EN`), not `1U << n`.
- No TIM14 on G4. New: TIM2, TIM5 (both **32-bit**), TIM8, TIM20. Drop the `#ifdef`s.
- IRQs are shared on G4: TIM1 update = `TIM1_UP_TIM16_IRQn`, TIM15 = `TIM1_BRK_TIM15_IRQn`, TIM6 shares with the DAC. Verify exact names in the header.
- **Bug fix:** the old `timer_get_irq` returned `(IRQn_Type)-1` as "not found", but -1 = `SysTick_IRQn`. Return a status and pass the IRQ through an out-pointer.
- **Bug fix:** SR bits are rc_w0. Use `TIMx->SR = ~TIM_SR_UIF;`, not `&=` (read-modify-write can drop other flags).
- `EGR = TIM_EGR_UG` (write-only, so `=` not `|=`). UG loads the buffered PSC/ARR and sets UIF → clear UIF before enabling UIE.
- `cnt` field: `uint16_t` → `uint32_t`. Check ARR ≤ 0xFFFF on 16-bit timers (`IS_TIM_32B_COUNTER_INSTANCE`).
- Add NULL checks; status names in uart style (`TIM_OK`, `TIM_ERROR_NULL`, `TIM_ERROR_INVALID_INSTANCE`, `TIM_ERROR_INVALID_CONFIG`).
- Check the NVIC enable result; loop index `uint32_t`. `timer_delay_s` is really "wait N updates" (only seconds at a 1 Hz update rate); the first wait may be a partial period.
- Math: count rate = 16 MHz/(PSC+1); update rate = count rate/(ARR+1). 1 Hz → PSC=15999, ARR=999.

### timer_pwm.c (mostly unchanged)
- CCMR/CCER layout same; OCxM is 4 bits (bits 6:4 + bit 16/24), so the old masks are fine. PWM1=6, PWM2=7.
- MOE list: add **TIM8 and TIM20** (every timer with BDTR needs MOE, or there's no output).
- Optionally swap magic masks for `TIM_CCMR1_OC1M`, `TIM_CCMR1_OC1PE`, `TIM_CCER_CC1E`.
- GPIO AF pin is configured in `device_instances.c`, not in the driver.
- f_pwm = 16 MHz/((PSC+1)(ARR+1)); e.g. PSC=0, ARR=799 → 20 kHz.
- First test idea: PC6 LED as TIM3_CH1 (AF2?) / TIM8_CH1 (AF4?). **Verify in datasheet.**

### timer_encoder.c
- **Bug:** never enables the RCC clock or sets ARR (relied on `timer_init` running first). Call `timer_init` first (PSC=0, ARR=max) or enable the clock itself.
- SMS is 4 bits (bit 16 = SMS[3]); old code already clears it. Keep modes 001/010/011; ignore the new G4 modes.
- Prefer TIM2/TIM5 (32-bit). With 16-bit timers, take the `int16_t` difference of CNT readings for wrap-safe velocity.
- Optionally replace magic bits with `TIM_CCMR1_CC1S`, `TIM_CCMR1_IC1F`, `TIM_SMCR_SMS`, `TIM_CR1_DIR`.

## Board integration
- Add the timer includes to `Board/device_instances.h`; put config instances + getters in `device_instances.c`.
- Call the inits in `board_init()`, add `BOARD_STATUS_*` errors. Optionally add `BOARD_TIM_CLK_HZ` to `board_config.h`.

## Testing
1. TIM2 at 1 Hz with interrupt → toggle LED in `TIM2_IRQHandler` (clear UIF **first** in the ISR).
2. PWM → dim the LED; check the frequency on a scope.
3. Encoder → print CNT/DIR over the console.
- Debug via the SFR view: RCC enable bit → CR1.CEN → DIER.UIE / CCER.CCxE → BDTR.MOE (advanced timers). If CNT moves but the ISR never runs, the handler name is misspelled (it lands in Default_Handler).

## Status
- [x] timer.c / timer.h: written by Claude (names checked against ST's cmsis-device-g4 header); needs a CubeIDE build + TIM2 1 Hz LED test
  - API change from G0: `timer_get_update_flag()` → `uint8_t timer_update_pending()`; `timer_init` takes `const TIM_Config_t *`
- [ ] timer_pwm
- [ ] timer_encoder
- [ ] board integration + tests
