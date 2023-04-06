const BUFFER_SIZE: usize = 4096*3;

pub struct RingBuffer {
    buffer: [u8; BUFFER_SIZE],
    head: usize,
    tail: usize,
    size: usize,
}

impl RingBuffer {
    pub const fn new() -> Self {
        Self {
            buffer: [0; BUFFER_SIZE],
            head: 0,
            tail: 0,
            size: 0,
        }
    }

    /**
     * Push an item into the ring buffer.
     */
    pub fn push(&mut self, item: u8) {
        if self.size == BUFFER_SIZE {
            panic!("Ring buffer is full!");
        }
        // push items into the circular buffer
        self.buffer[self.head] = item;
        self.head = (self.head+1) % BUFFER_SIZE;
        self.size += 1;
    }
    /**
     * Pop an item from the ring buffer.
     */
    pub fn pop(&mut self) -> Result<u8, ()> {
        if self.size == 0 {
            return Err(()) // Empty
        }
        let item = self.buffer[self.tail];
        self.tail = (self.tail+1) % BUFFER_SIZE;
        self.size -= 1;
        Ok(item)
    }
}