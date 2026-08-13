use core::ptr::NonNull;
use core::mem::MaybeUninit;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicUsize, Ordering, compiler_fence};

// Result Shorthand and Errors
type Result<T> = core::result::Result<T, RingBufferError>;
use RingBufferError::*;

#[derive(Debug)]
pub enum RingBufferError {
    Overflow,
    Underflow,
}

pub struct RingBuffer<T: Sized, const N: usize> {
    buf: [MaybeUninit<T>; N],
    head: AtomicUsize,
    tail: AtomicUsize,
}

impl<'a, T: Sized, const N: usize> RingBuffer<T, N> {

    pub const fn new() -> Self {
        Self {
            buf: unsafe { MaybeUninit::uninit().assume_init() },
            head: AtomicUsize::new(0),
            tail: AtomicUsize::new(0),
        }
    }

    pub fn split(&'a mut self) -> (RingProducer<'a, T, N>, RingConsumer<'a, T, N>) {
        unsafe {
            (
                RingProducer {
                    buf: NonNull::new_unchecked(self as *const _ as *mut _),
                    pd: PhantomData,
                },
                RingConsumer {
                    buf: NonNull::new_unchecked(self as *const _ as *mut _),
                    pd: PhantomData,
                },
            )
        }
    }
}

pub struct RingConsumer<'a, T: Sized, const N: usize> {
    buf: NonNull<RingBuffer<T, N>>,
    pd: PhantomData<&'a ()>,
}

impl <'a, T: Sized, const N: usize>RingConsumer<'a, T, N> {
    pub fn get(&mut self) -> Result<T> {
        let buf = unsafe { self.buf.as_mut() };

        let tail = buf.tail.load(Ordering::Relaxed);
        let head = buf.head.load(Ordering::Relaxed);

        // underflow condition
        if tail == head {
            return Err(Underflow);
        }

        let value = unsafe { buf.buf[tail].assume_init_read() };
        compiler_fence(Ordering::Release); // ensures data is read before tail is updated
        buf.tail.store((tail + 1) % N, Ordering::Relaxed);

        Ok(value)
    }
}

pub struct RingProducer<'a, T: Sized, const N: usize> {
    buf: NonNull<RingBuffer<T, N>>,
    pd: PhantomData<&'a ()>,
}

impl <'a, T: Sized, const N: usize>RingProducer<'a, T, N> {
    pub fn put(&mut self, value: T) -> Result<()> {
        let buf = unsafe { self.buf.as_mut() };

        let head = buf.head.load(Ordering::Relaxed);
        let tail = buf.tail.load(Ordering::Relaxed);

        // overflow condition
        if (head + 1) % N == tail {
            return Err(Overflow);
        }

        buf.buf[head] = MaybeUninit::new(value);
        compiler_fence(Ordering::Release); // ensures data is written before head is updated
        buf.head.store((head + 1) % N, Ordering::Relaxed);

        Ok(())
    }

    pub fn reset(&mut self) {
        let buf = unsafe { self.buf.as_mut() };
        buf.tail.store(0, Ordering::Relaxed);
        buf.head.store(0, Ordering::Relaxed);
    }
}