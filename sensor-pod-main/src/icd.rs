use defmt::Format;
use heapless::{String, Vec};
use serde::{Deserialize, Serialize};

use crate::adxl343::AccelData;

#[derive(Format, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub struct Request {
    #[serde(default)]
    pub id: Option<u32>,
    pub action: RequestAction,
}

#[derive(Format, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RequestAction {
    DfuWrite(DfuChunk),
    DfuMarkBooted,
    Poll,
    Status,
    SelfTest,
    SetHeartbeat(u8),
    SetActivityThreshold(u8),
    SetInactivityTime(u8),
    SetTapThreshold(u8),
    SetTapDuration(u8),
    ResetConfig,
    Reboot,
}

#[derive(Format, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub struct DfuChunk {
    pub version: String<10>,
    pub offset: usize,
    pub data: Vec<u8, 4096>,
    pub is_last: bool,
}

#[derive(Format, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub struct Response {
    pub id: u32,
    pub payload: ResponsePayload,
}

#[derive(Format, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ResponsePayload {
    DfuOffset(usize),
    Data(AccelData),
    Event(EventData),
    Status(Status),
}

#[derive(Format, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub struct Status {
    pub version: String<10>,
    pub uptime: u32,
    pub config: Config,
    pub temperature: i8,
}

#[derive(Format, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub struct EventData {
    #[serde(rename = "type")]
    pub event: Event,
    pub data: AccelData,
}

#[derive(Format, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Event {
    Activity,
    Inactivity,
    Tap,
}

#[derive(Format, Serialize, Deserialize)]
pub struct Config {
    pub heartbeat_interval: u8,
    pub activity_threshold: u8,
    pub inactivity_time: u8,
    pub tap_threshold: u8,
    pub tap_duration: u8,
}

impl From<crate::config::Config> for Config {
    fn from(cfg: crate::config::Config) -> Config {
        Config {
            heartbeat_interval: cfg.heartbeat_interval,
            activity_threshold: cfg.THRESH_ACT,
            inactivity_time: cfg.TIME_INACT,
            tap_threshold: cfg.THRESH_TAP,
            tap_duration: cfg.DUR,
        }
    }
}
