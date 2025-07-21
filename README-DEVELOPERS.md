<!-- SPDX-FileCopyrightText: 2021-2025 Smart Robotics Lab, Imperial College London, Technical University of Munich -->
<!-- SPDX-License-Identifier: BSD-3-Clause -->

# supereight2 developer guidelines



## Programming conventions

### TODO/XXX comments

TODO and XXX comments should follow the convention described
[here](https://drewdevault.com/2023/03/09/2023-03-09-Comment-or-no-comment.html).
In short:

* TODO is used when the implementation is wrong or incomplete. There might be a
  known bug, some edge case that isn't handled, etc.
* XXX is used when the implementation is correct but it might not be as
  efficient or readable as it could be. It might be using an inefficient
  algorithm, a deprecated API, the code could be simplified with some
  restructuring, etc.

Keep in mind that the above usage might not be consistent in all of supereight2
yet and there might be TODO comments that should have been XXX comments.



## Using Eigen

**The
[Eigen alignment issues page](https://eigen.tuxfamily.org/dox/group__DenseMatrixManipulation__Alignement.html)
is a mandatory read before starting work on supereight2.**

### Don't use make\_shared/make\_unique

Alignment issues can arise when initializing `std::shared_ptr`/`std::unique_ptr`
to objects containing Eigen members with `make_shared`/`make_unique`. Use the
respective constructors instead.

``` cpp
// CORRECT, using the std::shared_ptr/std::unique_ptr constructor
x = std::shared_ptr<T>(new T(constructor_parameter));

// WRONG, using std::make_shared/std::make_unique
x = std::make_shared<T>(constructor_parameter);
```

Most C++ implementations seem to ignore the overloaded operator new defined by
Eigen. See [here](https://gitlab.com/libeigen/eigen/-/issues/1049) for details.



## Using OpenMP

### Declare loop variables inside the loop.

``` cpp
// CORRECT, loop variable declared inside the loop.
#pragma omp parallel for
for (int i = 0; i < 8; i++) ...

// WRONG, loop variable declared outside the loop.
int i;
#pragma omp parallel for
for (i = 0; i < 8; i++) ...
```

### Use named critical sections

The OpenMP `critical` directive is used to define a section of code that can
only be executed by a single thread at a time. Directives with the same name
share the same lock. Directives without a name all share the same lock. You
should always use named `critical` directives to avoid using the same lock in
multiple parts of the code.

``` cpp
// CORRECT, named critical section.
#pragma omp critical(stuff_lock)
{
    do_stuff();
}

// WRONG, unnamed critical section, shared lock.
#pragma omp critical
{
    do_stuff();
}
```



## Workflows

### Running more extensive tests

Run the testing script
[`scripts/supereight-test.bash`](./scripts/supereight-test.bash), passing it a
directory where the output logs will be written, like so:

``` sh
./scripts/supereight-test.bash /tmp/se2results
```

It will test several compilers and sanitizers.

### Making a release

Follow the instructions in
[`doc/supereight2_release_procedure.md`](doc/supereight2_release_procedure.md).



## Useful tools and options

### Enabling assertions in release builds

To enable asserts in Release and RelWithDebInfo builds either pass
`-DSE_ASSERTS=ON` to `cmake` or set `SE_ASSERTS` to `ON` in
[`CMakeLists.txt`](CMakeLists.txt).

### supereight2\_sizes

The `supereight2_sizes` executable prints the sizes of the most common
supereight2 structures for all possible map configurations. The output is in TSV
([tab-separated-values](https://www.iana.org/assignments/media-types/text/tab-separated-values))).
This tool allows seeing if changing the order of some struct's data members
changed its size. See also
[here](http://www.catb.org/esr/structure-packing/#_c).



## Known issues

### supereight2 is slower than expected

If you have a CPU with a large number of threads, consider limiting them to
around 10 or so by setting the `OMP_NUM_THREADS` environment variable. You can
also run `./scripts/supereight2-optimal-num-threads.sh PROGRAM CONFIG` to find
the optimal number of OpenMP threads for running supereight2 on your computer.

### Building in debug mode fails with GCC 9

The following build error (and similar ones for `LeicaLidar` and `OusterLidar`)
is due to the GCC 9 bug described
[here](https://gcc.gnu.org/bugzilla/show_bug.cgi?id=94903). It's fixed in GCC
10.

``` text
during RTL pass: expand
supereight2/src/sensor/pinhole_camera.cpp: In member function ‘se::PinholeCamera::Config se::PinholeCamera::Config::operator/(float) const’:
supereight2/src/sensor/pinhole_camera.cpp:38:5: internal compiler error: in assign_temp, at function.c:982
   38 |     };
      |     ^
0x7f69dbf03082 __libc_start_main
        ../csu/libc-start.c:308
Please submit a full bug report,
with preprocessed source if appropriate.
Please include the complete backtrace with any bug report.
See <file:///usr/share/doc/gcc-9/README.Bugs> for instructions
```

### The address sanitizer (ASAN) detects a leak from libtbb

This error is not in our code, just ignore it.
