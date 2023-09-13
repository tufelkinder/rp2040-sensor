use embassy_rp::uart::Error;
use embedded_io_async::{Read, Write, WriteAllError};
use serde::{Deserialize, Serialize};
use serde_json_core::{from_slice, to_slice};

/// Serialize and write to i/o
pub async fn write_json<T: Serialize, const BUF_SIZE: usize>(
    tx: &mut impl Write<Error = Error>,
    msg: &T,
) -> Result<(), WriteAllError<Error>> {
    let mut data = [b'\n'; BUF_SIZE];
    let n = to_slice(&msg, &mut data).unwrap();
    tx.write_all(&data[..n + 1]).await
}

/// Read i/o and deserialize
pub async fn read_json_stream<T, Fut, const N: usize>(mut rx: impl Read, f: impl Fn(T) -> Fut)
where
    T: for<'de> Deserialize<'de>,
    Fut: futures::Future<Output = ()>,
{
    let mut json_buf: JsonAccumulator<N> = JsonAccumulator::new();
    let mut raw_buf = [0; N];
    loop {
        if let Ok(n) = rx.read(&mut raw_buf).await {
            let buf = &raw_buf[..n];
            let mut window = &buf[..];
            'json: while !window.is_empty() {
                window = match json_buf.feed::<T>(&window) {
                    FeedResult::Consumed => break 'json,
                    FeedResult::OverFull(new_wind) => new_wind,
                    FeedResult::DeserError(new_wind) => new_wind,
                    FeedResult::Success { data, remaining } => {
                        f(data).await;
                        remaining
                    }
                };
            }
        }
    }
}

#[derive(defmt::Format)]
pub struct JsonAccumulator<const N: usize> {
    buf: [u8; N],
    idx: usize,
}

/// The result of feeding the accumulator.
#[derive(defmt::Format)]
pub enum FeedResult<'a, T> {
    /// Consumed all data, still pending.
    Consumed,

    /// Buffer was filled. Contains remaining section of input, if any.
    OverFull(&'a [u8]),

    /// Reached end of chunk, but deserialization failed. Contains remaining section of input, if.
    /// any
    DeserError(&'a [u8]),

    /// Deserialization complete. Contains deserialized data and remaining section of input, if any.
    Success {
        /// Deserialize data.
        data: T,

        /// Remaining data left in the buffer after deserializing.
        remaining: &'a [u8],
    },
}

impl<const N: usize> JsonAccumulator<N> {
    /// Create a new accumulator.
    pub const fn new() -> Self {
        JsonAccumulator { buf: [0; N], idx: 0 }
    }

    /// Appends data to the internal buffer and attempts to deserialize the accumulated data into
    /// `T`.
    #[inline]
    pub fn feed<'a, T>(&mut self, input: &'a [u8]) -> FeedResult<'a, T>
    where
        T: for<'de> Deserialize<'de>,
    {
        self.feed_ref(input)
    }

    /// Appends data to the internal buffer and attempts to deserialize the accumulated data into
    /// `T`.
    ///
    /// This differs from feed, as it allows the `T` to reference data within the internal buffer, but
    /// mutably borrows the accumulator for the lifetime of the deserialization.
    /// If `T` does not require the reference, the borrow of `self` ends at the end of the function.
    pub fn feed_ref<'de, 'a, T>(&'de mut self, input: &'a [u8]) -> FeedResult<'a, T>
    where
        T: Deserialize<'de>,
    {
        if input.is_empty() {
            return FeedResult::Consumed;
        }

        let eom_pos = input.iter().position(|&i| i == b'\n' || i == b'\r');

        if let Some(n) = eom_pos {
            // Yes! We have an end of message here.
            // Add one to include the zero in the "take" portion
            // of the buffer, rather than in "release".
            let (take, release) = input.split_at(n + 1);

            // Does it fit?
            if (self.idx + n) <= N {
                // Yes - add to array
                self.extend_unchecked(take);
                let retval = match from_slice::<T>(&mut self.buf[..self.idx]) {
                    Ok((t, _)) => FeedResult::Success {
                        data: t,
                        remaining: release,
                    },
                    Err(_) => FeedResult::DeserError(release),
                };
                self.idx = 0;
                retval
            } else {
                self.idx = 0;
                FeedResult::OverFull(release)
            }
        } else {
            // Does it fit?
            if (self.idx + input.len()) > N {
                // No
                let new_start = N - self.idx;
                self.idx = 0;
                FeedResult::OverFull(&input[new_start..])
            } else {
                // Yes
                self.extend_unchecked(input);
                FeedResult::Consumed
            }
        }
    }

    /// Extend the internal buffer with the given input.
    ///
    /// # Panics
    ///
    /// Will panic if the input does not fit in the internal buffer.
    fn extend_unchecked(&mut self, input: &[u8]) {
        let new_end = self.idx + input.len();
        self.buf[self.idx..new_end].copy_from_slice(input);
        self.idx = new_end;
    }
}
