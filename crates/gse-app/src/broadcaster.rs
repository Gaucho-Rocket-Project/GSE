//! Fan-out of state snapshots and log lines to the browser (SSE).

use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::Mutex;

use crate::logging::LogEntry;

#[derive(Clone, Debug)]
pub enum EventMsg {
    State(String),
    Log(String),
}

pub struct Broadcaster {
    subscribers: Mutex<Vec<Sender<EventMsg>>>,
    last_state: Mutex<String>,
}

impl Broadcaster {
    pub fn new(initial_state: String) -> Self {
        Broadcaster {
            subscribers: Mutex::new(Vec::new()),
            last_state: Mutex::new(initial_state),
        }
    }

    fn lock<T>(mutex: &Mutex<T>) -> std::sync::MutexGuard<'_, T> {
        mutex.lock().unwrap_or_else(|e| e.into_inner())
    }

    /// Register a new SSE client. Returns the receiver and the latest state so
    /// the caller can send it immediately (browsers may connect before the next
    /// engine tick).
    pub fn subscribe(&self) -> (Receiver<EventMsg>, String) {
        let (tx, rx) = mpsc::channel();
        let last = Self::lock(&self.last_state).clone();
        Self::lock(&self.subscribers).push(tx);
        (rx, last)
    }

    pub fn state(&self, json: &str) {
        *Self::lock(&self.last_state) = json.to_string();
        self.send(EventMsg::State(json.to_string()));
    }

    pub fn log(&self, entry: &LogEntry) {
        self.send(EventMsg::Log(entry.to_json()));
    }

    pub fn subscriber_count(&self) -> usize {
        Self::lock(&self.subscribers).len()
    }

    fn send(&self, message: EventMsg) {
        let mut subs = Self::lock(&self.subscribers);
        subs.retain(|tx| tx.send(message.clone()).is_ok());
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::logging::{Level, Source};

    #[test]
    fn delivers_state_and_logs() {
        let hub = Broadcaster::new("{\"rev\":0}".to_string());
        let (rx, initial) = hub.subscribe();
        assert_eq!(initial, "{\"rev\":0}");
        hub.state("{\"rev\":1}");
        let entry = LogEntry {
            id: 1,
            epoch_ms: 0,
            level: Level::Info,
            source: Source::App,
            message: "hello".into(),
            code: None,
        };
        hub.log(&entry);
        match rx.recv().unwrap() {
            EventMsg::State(s) => assert_eq!(s, "{\"rev\":1}"),
            _ => panic!("expected state"),
        }
        match rx.recv().unwrap() {
            EventMsg::Log(s) => assert!(s.contains("hello")),
            _ => panic!("expected log"),
        }
        assert_eq!(hub.subscriber_count(), 1);
    }

    #[test]
    fn drops_closed_subscribers() {
        let hub = Broadcaster::new("{}".to_string());
        let (rx, _) = hub.subscribe();
        drop(rx);
        hub.state("{}");
        assert_eq!(hub.subscriber_count(), 0);
    }
}
