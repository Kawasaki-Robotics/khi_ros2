// Copyright 2025 Kawasaki Heavy Industries, Ltd.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <iconv.h>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>

#include "khi_hardware/khi_driver.hpp"

namespace khi_hardware
{
KhiDriver::~KhiDriver() {}

/**
 * @brief Changes the character encoding.
 * @param input Input string
 * @param dst Destination character encoding
 * @param src Source character encoding
 * @return std::string Output string
 */
std::string KhiDriver::convert_encoding(
  const char * input, const char * dst, const char * src) const
{
  constexpr intptr_t iconv_error = -1;
  iconv_t cd = iconv_open(dst, src);
  if (reinterpret_cast<intptr_t>(cd) == iconv_error)
  {
    return std::string(input);
  }

  size_t inbytesleft = strlen(input);
  size_t outbytesleft = inbytesleft * 4;  // UTF-8 may use up to 4 bytes per character
  std::string output(outbytesleft, '\0');

  char * inbuf = const_cast<char *>(input);
  char * outbuf = &output[0];

  if (iconv(cd, &inbuf, &inbytesleft, &outbuf, &outbytesleft) == static_cast<size_t>(-1))
  {
    iconv_close(cd);
    return std::string(input);
  }

  output.resize(output.size() - outbytesleft);  // Adjust the size of the output string
  iconv_close(cd);

  return output;
}
}  // namespace khi_hardware
