pub struct RingBuffer {
    buffer: [u8; 4096],
    head: usize,
    tail: usize,
    size: u16,
}

impl RingBuffer {
    pub const fn new() -> Self {
        Self {
            buffer: [0; 4096],
            head: 0,
            tail: 0,
            size: 0,
        }
    }

    /**
     * Push an item into the ring buffer.
     */
    pub fn push(&mut self, item: u8) {
        // push items into the circular buffer
        self.buffer[self.tail] = item;
        self.tail = (self.tail + 1) % (4096);
        self.size += 1;
    }
    /**
     * Pop an item from the ring buffer.
     */
    pub fn pop(&mut self) -> Result<u8, ()> {
        if self.size == 0 {
            return Err(());
        }
        let item = self.buffer[self.head];
        self.head = (self.head + 1) % (4096);
        self.size -= 1;
        Ok(item)
    }
}