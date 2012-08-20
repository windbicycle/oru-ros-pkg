/* This file is part of the Pangolin Project.
 * http://github.com/stevenlovegrove/Pangolin
 *
 * Copyright (c) 2011 Steven Lovegrove
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use,
 * copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following
 * conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef PANGOLIN_THREADED_WRITE_H
#define PANGOLIN_THREADED_WRITE_H

#include <iostream>
#include <streambuf>
#include <fstream>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>

namespace pangolin
{

class threadedfilebuf : public std::streambuf
{
public:
    threadedfilebuf(const std::string& filename, unsigned int buffer_size_bytes);
    ~threadedfilebuf();

    void operator()();

protected:
    //! Override streambuf::xsputn for asynchronous write
    std::streamsize xsputn(const char * s, std::streamsize n);

    std::filebuf file;
    char* mem_buffer;
    int mem_size;
    int mem_max_size;
    int mem_start;
    int mem_end;

    boost::mutex update_mutex;
    boost::condition_variable cond_queued;
    boost::condition_variable cond_dequeued;
    boost::thread write_thread;
};

}


#endif // PANGOLIN_THREADED_WRITE_H
