/// All events need to implement both the [`core::fmt::Display`] trait and the [`defmt::Format`]
/// trait. Since they are both similar to implement, this macro helps with this task.
macro_rules! display_event {
    ($( [$e:ident, $s:literal$(,)? $( $p:ident ),* ] ),*) => {
        impl core::fmt::Display for Event {
            fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
                match self {
                    $ (
                        Event::$e($($p),*) => write!(f, $s, $($p),*),
                    )*
                }
            }
        }

        impl defmt::Format for Event {
            fn format(&self, f: defmt::Formatter) {
                match self {
                    $ (
                        Event::$e($($p),*) => defmt::write!(f, $s, $($p),*),
                    )*
                }
            }
        }
    };
}

/// All errors need to implement both the [`core::fmt::Display`] trait and the [`defmt::Format`]
/// trait. Since they are both similar to implement, this macro helps with this task.
macro_rules! display_context {
    ($( [$e:ident, $s:literal] ),*) => {
        impl core::fmt::Display for ErrorContext {
            fn fmt(&self, f: &mut Formatter<'_>) -> core::fmt::Result {
                match self {
                    $ (
                        ErrorContext::$e => write!(f, $s),
                    )*
                }
            }
        }

        impl defmt::Format for ErrorContext {
            fn format(&self, f: defmt::Formatter) {
                match self {
                    $ (
                        ErrorContext::$e => defmt::write!(f, $s),
                    )*
                }
            }
        }
    };
}

pub(crate) use display_context;
pub(crate) use display_event;
