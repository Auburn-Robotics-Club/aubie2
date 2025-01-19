//! Kernel Logger Implementation

extern crate alloc;

use core::time::Duration;

use log::{max_level, set_logger, set_max_level, LevelFilter, Log, Metadata, SetLoggerError};

const ESCAPES: [Option<&str>; 6] = [
    None,             // Default foreground
    Some("\x1B[31m"), // Error (red)
    Some("\x1B[33m"), // Warn (yellow)
    Some("\x1B[34m"), // Info (blue)
    Some("\x1B[36m"), // Debug (cyan)
    Some("\x1B[37m"), // Trace (white)
];

pub struct SerialLogger;

impl SerialLogger {
    pub fn init(&'static self, level: LevelFilter) -> Result<(), SetLoggerError> {
        set_logger(self)?;
        set_max_level(level);

        Ok(())
    }
}

impl Log for SerialLogger {
    fn enabled(&self, metadata: &Metadata<'_>) -> bool {
        metadata.level() <= max_level()
    }

    fn log(&self, record: &log::Record<'_>) {
        if self.enabled(record.metadata()) {
            let timestamp = Duration::from_micros(unsafe { vex_sdk::vexSystemHighResTimeGet() });
            let mins = timestamp.as_secs() / 60;
            let submin_secs = timestamp.as_secs() % 60;

            vexide::core::println!(
                "{:02}:{:02}:{:02} {}[{}]\x1B[0m {}",
                mins,
                submin_secs,
                timestamp.subsec_millis(),
                ESCAPES[record.level() as usize].unwrap_or_default(),
                record.level(),
                record.args()
            );
        }
    }

    fn flush(&self) {}
}
