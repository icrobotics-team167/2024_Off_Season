The classes in this package are pretty cluttered due to an extra layer of abstraction (IO ->
Abstract -> Impl instead of IO -> Impl) and more implementation classes. It feels really
Java-brained and dumb, and that's because it is.

However, I saw this as neccesary, due to REVLib's sim support still kinda sucking even after the 
2025 library rework, with the documentation still bad. Unlike Phoenix's well documented sim 
support that don't require separation of real and sim, as far as I can tell, REVLib still 
requires you to separate logic between real and sim. In order to avoid code duplication whilst 
still maintaining the separation between deterministic and non-deterministic code, I created the 
abstract classes to share code between the sim and real implementations. Maybe this will get 
cleaned up when REV releases documentation and/or an update that doesn't require real/sim 
separation.

Compiling will also throw warnings about deprecation. The feedforward calculate methods that 
use doubles instead of the Units API are deprecated (See
[allwpilib#7280](https://github.com/wpilibsuite/allwpilib/issues/7280) for reasoning) but we 
want to avoid using the Units API at all costs. Unlike the C++ Units API which has 0 runtime
performance costs due to being optimized out during compile time, the Java Units API imposes a 
heavy memory cost as each use of the Units API adds 1 object allocation, which very rapidly 
adds up to hundreds of allocations per tick, which is very bad for the extremely 
RAM-constrained RoboRIO.

2027 cannot come fast enough.

Tada
