/*!
MIT License

Copyright (c) 2022 Gotelli Andrea

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "cpr_visual/shape_publisher.h"
#include <iostream>
#include <map>

namespace cpr_visual
{

ShapePublisher::ShapePublisher(std::chrono::milliseconds timeout)
  : sock(ctx, zmq::socket_type::pair), timeout(timeout)
{
  sock.setsockopt( ZMQ_LINGER, 0 );
  sock.bind(socket_name);
}

ShapePublisher::~ShapePublisher()
{
  sock.close();
  ctx.close();
}


void ShapePublisher::publish(const ShapeMsg &msg)
{
  static zmq::pollitem_t poll_out{nullptr, 0, ZMQ_POLLOUT, 0};
  poll_out.socket = static_cast<void*>(sock);
  zmq::poll(&poll_out, 1, timeout);

  if(poll_out.revents & ZMQ_POLLOUT)
  {
    zmq::message_t zmsg(&msg, sizeof(msg));
    sock.send(zmsg, zmq::send_flags::none);
  }
}


}  //  END namespace cpr_visual
