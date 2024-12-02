Compiling will throw warnings about deprecation. The feedforward calculate methods that use
doubles instead of the Units API are deprecated (See
[allwpilib#7280](https://github.com/wpilibsuite/allwpilib/issues/7280) and
[Tyler's comment in allwpilib#6647](https://github.com/wpilibsuite/allwpilib/pull/6647#discussion_r1633591865)
for reasoning) but we want to avoid using the Units API at all costs. Unlike the C++ Units API
which has 0 runtime performance costs due to being optimized out during compile time, the Java
Units API imposes a heavy memory cost as each use of the Units API adds 1 object allocation,
which very rapidly adds up to hundreds of allocations per tick, which is very bad for the
extremely RAM-constrained RoboRIO.

2027 cannot come fast enough.

Tada
