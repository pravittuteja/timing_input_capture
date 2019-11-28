Using the STM32 Discovery board design and implement an embedded, bare-metal (no operating
system) program that will display a list of counts for one thousand rising edge pulse inter-arrival times.
The inter-arrival time between pulses is expected to average around 1.0 millisecond, but the listing
should represent the range of 101 “buckets” (one bucket per microsecond) between the default
values of 950 and 1050 microseconds. However, these upper and lower limits must be user
configurable via the virtual terminal.


The list of counts should be in ascending time order. A typical display would have 3 lines like
this:
998 5
999 950
1000 45


