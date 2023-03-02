#[derive(Copy, Clone, Default)]
pub struct Command(u8, u8);

impl Command {
    pub fn new(modifier: u8, key: u8) -> Self {
        Self(modifier, key)
    }
}

pub struct CommandRingBuffer<const N: usize> {
    buffer: [Option<Command>; N],
    current_write_index: usize,
    current_read_index: usize,
}

impl<const N: usize> CommandRingBuffer<N> {
    pub fn new() -> Self {
        CommandRingBuffer {
            buffer: [None; N],
            current_write_index: 0,
            current_read_index: 0,
        }
    }

    /// Adds an item to the ring buffer and returns true if this consumed an unused older item
    /// (i.e. if the read pointer was moved forwards)
    pub fn push(&mut self, cmd: Command) -> bool {
        self.buffer[self.current_write_index] = Some(cmd);

        if self.current_write_index == self.current_read_index {
            self.current_read_index = (self.current_read_index + 1) % N;
            self.current_write_index = (self.current_write_index + 1) % N;
            true
        } else {
            self.current_write_index = (self.current_write_index + 1) % N;
            false
        }
    }

    /// Pops the current item off the ring buffer. If the current item is None, i.e. unset, the
    /// read position isn't moved. If the current item is Some, or if we aren't at the same place
    /// as the ring buffer cursor, then we move the read position forward.
    pub fn pop(&mut self) -> Option<Command> {
        let result = self.buffer[self.current_read_index].take();

        if result.is_some() || self.current_read_index != self.current_write_index {
            self.current_read_index = (self.current_read_index + 1) % N;
        }

        result
    }
}
