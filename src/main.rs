use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::*;
use esp_idf_hal::ledc::*;
use esp_idf_hal::peripherals::Peripherals;
use esp_idf_hal::prelude::Hertz;
use std::sync::mpsc;
use std::sync::Arc;
use std::sync::Mutex;
use std::thread;
use std::time::*;

// デバイス情報取得
struct Device {
    input_button: AnyIOPin, // ボタン入力ピン (プルアップする都合上AnyInputPinではなくAnyIOPin)
    output_leds: [AnyOutputPin; MotorPowerManager::POWER_INDICATOR_MAX], // 出力確認用LED出力ピン
    motor_pwm_pin: AnyOutputPin, // モーター制御用PWM出力ピン
    motor_pwm_channel: CHANNEL0, // PWMチャンネル
    motor_pwm_ledc_timer: TIMER0, // PWMタイマー
}

impl Device {
    pub fn new() -> anyhow::Result<Self> {
        let peripherals = Peripherals::take()?;
        Ok(Device {
            input_button: peripherals.pins.gpio4.downgrade(),
            output_leds: [
                peripherals.pins.gpio1.downgrade_output(),
                peripherals.pins.gpio2.downgrade_output(),
                peripherals.pins.gpio3.downgrade_output(),
            ],
            motor_pwm_pin: peripherals.pins.gpio0.downgrade_output(),
            motor_pwm_channel: peripherals.ledc.channel0,
            motor_pwm_ledc_timer: peripherals.ledc.timer0,
        })
    }
}

// ボタン用入力イベントタイプ
enum ButtonEventType {
    Push,
    LongPush,
}

// ボタン入力監視
struct ButtonEventWatcher {
    input_button: Arc<Mutex<PinDriver<'static, AnyIOPin, Input>>>, // 入力ピンドライバ
    event_sender: mpsc::Sender<ButtonEventType>,                   // 入力イベント送信者
    is_exec_event: Mutex<bool>,                                    // ボタンイベント状況
    last_button_state: Mutex<bool>,                                // 最終ボタン状況
    last_push_time: Mutex<u128>, // チャタリング対策・長押し判定用時刻
}

impl ButtonEventWatcher {
    const WATCH_INTERVAL_MS: u32 = 10; // 検知間隔(ms)
    const DEBOUNCE_DELAY_MS: u128 = 30; // チャタリングのディレイ時間(ms)
    const LONG_PRESS_MS: u128 = 1000; // 長押し判定時間（ms）

    fn new(
        input_pin: AnyIOPin,
        event_sender: mpsc::Sender<ButtonEventType>,
    ) -> anyhow::Result<Self> {
        let input_button = Arc::new(Mutex::new(PinDriver::input(input_pin)?));
        input_button.lock().unwrap().set_pull(Pull::Up)?;

        Ok(Self {
            input_button,
            event_sender,
            is_exec_event: Mutex::new(true),
            last_button_state: Mutex::new(false),
            last_push_time: Mutex::new(0),
        })
    }

    fn exec(&self) -> anyhow::Result<()> {
        loop {
            self.update()?;
            FreeRtos::delay_ms(Self::WATCH_INTERVAL_MS);
        }
    }

    fn update(&self) -> anyhow::Result<()> {
        let now_millis = SystemTime::now().duration_since(UNIX_EPOCH)?.as_millis();
        let button_state = self.input_button.lock().unwrap().is_high();
        let mut last_button_state = self.last_button_state.lock().unwrap();
        let mut last_push_time = self.last_push_time.lock().unwrap();
        let mut is_exec_event = self.is_exec_event.lock().unwrap();

        // ボタンがおされっぱなしの状態
        if !button_state
            && !*last_button_state
            && (*last_push_time + Self::LONG_PRESS_MS) <= now_millis
            && !*is_exec_event
        {
            log::info!("Button Long Push");
            self.event_sender.send(ButtonEventType::LongPush)?;
            *is_exec_event = true;
            return Ok(());
        }

        if button_state == *last_button_state {
            // ボタンに変化なしの場合は終了
            return Ok(());
        }

        // ボタンの変化がある場合
        *last_button_state = button_state;
        if !button_state {
            // ボタンが押された
            *last_push_time = now_millis;
            *is_exec_event = false;
        } else {
            // ボタンが離された
            if Self::DEBOUNCE_DELAY_MS < (now_millis - *last_push_time) && !*is_exec_event {
                log::info!("Button Push");
                self.event_sender.send(ButtonEventType::Push)?;
                *is_exec_event = true;
            }
        }
        Ok(())
    }
}

// モーター電力管理
struct MotorPowerManager {
    power_level: Mutex<usize>, // パワーレベル
    output_led_drivers: Vec<Arc<Mutex<PinDriver<'static, AnyOutputPin, Output>>>>, // 状況出力用LEDピンデバイス
    motor_pwm: Arc<Mutex<LedcDriver<'static>>>, // モーター用LEDCドライバ
    event_receiver: Mutex<mpsc::Receiver<ButtonEventType>>, // イベント受信者
}

impl MotorPowerManager {
    const POWER_LEVEL_MAX: usize = 4; // OFFを含めたパワーレベル数
    const POWER_LEVEL_PWM_TABLE: [f32; Self::POWER_LEVEL_MAX] = [0.0, 0.1, 0.3, 1.0]; // レベル別PWM設定テーブル
    const POWER_INDICATOR_MAX: usize = Self::POWER_LEVEL_MAX - 1; // LEDインジケーター数
    const PWM_FREQUENCY: Hertz = Hertz(25_000); // PWM周波数

    fn new(
        motor_pwm_pin: AnyOutputPin,
        motor_pwm_channel: CHANNEL0,
        motor_pwm_ledc_timer: TIMER0,
        output_leds: [AnyOutputPin; Self::POWER_INDICATOR_MAX],
        event_receiver: mpsc::Receiver<ButtonEventType>,
    ) -> anyhow::Result<Self> {
        let mut drivers: Vec<Arc<Mutex<PinDriver<'static, AnyOutputPin, Output>>>> = Vec::new();
        for pin in output_leds.into_iter() {
            drivers.push(Arc::new(Mutex::new(PinDriver::output(pin)?)));
        }

        Ok(Self {
            power_level: Mutex::new(0),
            output_led_drivers: drivers,
            motor_pwm: Arc::new(Mutex::new(LedcDriver::new(
                motor_pwm_channel,
                LedcTimerDriver::new(
                    motor_pwm_ledc_timer,
                    &config::TimerConfig::new().frequency(Self::PWM_FREQUENCY),
                )?,
                motor_pwm_pin,
            )?)),
            event_receiver: Mutex::new(event_receiver),
        })
    }

    fn exec(&self) -> anyhow::Result<()> {
        loop {
            match self.event_receiver.lock().unwrap().recv()? {
                ButtonEventType::Push => {
                    // パワーレベルのレベルローテート
                    if self.is_power_on() {
                        self.rotate_power_level()?;
                    }
                }
                ButtonEventType::LongPush => {
                    // ON/OFF
                    if self.is_power_on() {
                        self.power_off()?;
                    } else {
                        self.power_on()?;
                    }
                }
            }
        }
    }

    fn power_on(&self) -> anyhow::Result<()> {
        log::info!("Power On");
        let mut power_level = self.power_level.lock().unwrap();
        *power_level = 1;
        self.set_power_level(*power_level)
    }

    fn power_off(&self) -> anyhow::Result<()> {
        log::info!("Power Off");
        let mut power_level = self.power_level.lock().unwrap();
        *power_level = 0; // 0はOFF
        self.set_power_level(*power_level)
    }

    fn is_power_on(&self) -> bool {
        *self.power_level.lock().unwrap() != 0
    }

    fn rotate_power_level(&self) -> anyhow::Result<()> {
        let mut power_level = self.power_level.lock().unwrap();
        *power_level = ((*power_level as i32 % (Self::POWER_LEVEL_MAX as i32 - 1)) + 1) as usize;
        log::info!("RotatePowerLevel powerLevel:{}", *power_level);
        self.set_power_level(*power_level)
    }

    fn set_power_level(&self, mut level: usize) -> anyhow::Result<()> {
        if Self::POWER_LEVEL_MAX <= level {
            // 範囲外の場合は強制OFF
            level = 0;
        }

        // パワーインジケーター設定(LED消灯・点灯制御)
        for pin_idx in 0..Self::POWER_INDICATOR_MAX {
            self.output_led_drivers[pin_idx].lock().unwrap().set_level(
                if (pin_idx + 1) <= level {
                    Level::High
                } else {
                    Level::Low
                },
            )?;
        }

        // PWM出力変更
        let mut motor_pwm = self.motor_pwm.lock().unwrap();
        let max_duty = motor_pwm.get_max_duty();
        let duty_rate = (max_duty as f32 * Self::POWER_LEVEL_PWM_TABLE[level]) as u32;
        //log::info!("Duty Val:{} Max:{}", duty_rate, max_duty);
        motor_pwm.set_duty(duty_rate)?;

        Ok(())
    }
}

struct Controller {
    button_event_watcher: ButtonEventWatcher,
    motor_power_manager: MotorPowerManager,
}

impl Controller {
    pub fn new(device: Device) -> anyhow::Result<Self> {
        let (sender, receiver) = mpsc::channel::<ButtonEventType>();

        Ok(Self {
            button_event_watcher: ButtonEventWatcher::new(device.input_button, sender)?,
            motor_power_manager: MotorPowerManager::new(
                device.motor_pwm_pin,
                device.motor_pwm_channel,
                device.motor_pwm_ledc_timer,
                device.output_leds,
                receiver,
            )?,
        })
    }

    fn start(&self) -> anyhow::Result<()> {
        log::info!("Start");

        thread::scope(|scope| -> anyhow::Result<()> {
            let button_event_watcher_handle = scope.spawn(|| -> anyhow::Result<()> {
                log::info!("Start Button Event Thread");
                self.button_event_watcher.exec()
            });
            let motor_power_manager_handle = scope.spawn(|| -> anyhow::Result<()> {
                log::info!("Start Motor Power Manager Thread");
                self.motor_power_manager.exec()
            });
            button_event_watcher_handle.join().unwrap()?;
            motor_power_manager_handle.join().unwrap()
        })
    }
}

fn main() -> anyhow::Result<()> {
    esp_idf_svc::sys::link_patches();
    esp_idf_svc::log::EspLogger::initialize_default();

    Controller::new(Device::new()?)?.start()
}
