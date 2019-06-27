/*! \page page_buffer Buffer
<H3>Boost circular buffer</H3>
The buffer is based on a thread-safe <a href="http://www.boost.org/doc/libs/1_54_0/libs/circular_buffer/doc/circular_buffer.html#boundedbuffer">circular buffer implementation of boost</a> .

<H3>Buffer Size</H3>
The buffer size determines the initial buffer size.

<H3>Buffer Types</H3>
The buffer can have one of three different types.
<ul>
  <li> <B>Fixed Size: </B> Once the buffer is full, no elements can be added to the buffer.</li>
  <li> <B>Looping: </B> Once the buffer is full, the oldest element is overwritten by the new one.</li>
  <li> <B>Growing: </B> Once the buffer is full, the buffer is resized in an exponential manor.<BR>
                    <B>Increasing buffer size (reallocating) is very time inefficient, in time critical application do not use this buffer type!</B></li>
</ul>
*/
