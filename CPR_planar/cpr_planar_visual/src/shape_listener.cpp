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




#include <cpr_planar_visual/shape_listener.h>
#include "cpr_planar_visual/zmq.hpp"

namespace cpr_planar_visual
{

ShapeListener::ShapeListener(std::chrono::milliseconds timeout) : timeout(timeout)
{
  //  Define the thread to dedicate to the listening process
  listener = std::thread([&](){listening_loop();});

}

ShapeListener::~ShapeListener()
{
  running = false;
  listener.join();
}

void ShapeListener::listening_loop()
{
  zmq::context_t ctx;
  zmq::socket_t sock(ctx, zmq::socket_type::pair);
  sock.setsockopt( ZMQ_LINGER, 0 );

  sock.connect(socket_name);

  zmq::pollitem_t poll_in{nullptr, 0, ZMQ_POLLIN, 0};
  poll_in.socket = static_cast<void*>(sock);

  while(running)
  {
    zmq::poll(&poll_in, 1, timeout);

    if(poll_in.revents & ZMQ_POLLIN)
    {
      zmq::message_t zmsg;
      (void)sock.recv(zmsg);
      msg = *(static_cast<ShapeMsg*>(zmsg.data()));
    }
  }
  sock.close();
  ctx.close();
}


} //  END namespace cpr_planar_visual
