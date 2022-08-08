/* Copyright (C) 2020-2021 Oxan van Leeuwen
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#pragma once

#include "esphome/core/version.h"
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"

#include <memory>
#include <string>
#include <vector>

#ifdef ARDUINO_ARCH_ESP8266
#include <ESPAsyncTCP.h>
#else
// AsyncTCP.h includes parts of freertos, which require FreeRTOS.h header to be included first
#include <freertos/FreeRTOS.h>
#include <AsyncTCP.h>
#endif

using esphome::uart::UARTComponent;

using SendCb = std::function<void(const std::vector<uint8_t> &)>;
using OnUartDataCb = std::function<void(const std::vector<uint8_t> &, SendCb)>;

void onData(const std::vector<uint8_t> &input, SendCb send) {
  static std::vector<uint8_t> buf;
  for (uint8_t x : input) {
    const size_t idx = buf.size();
    if (idx == 0 && x != 0xAA)
      continue;
    if (idx == 1 && x < 10) {
      buf.clear();
      continue;
    }
    buf.push_back(x);
    if (idx < 10 || idx != buf[1])
      continue;
    uint8_t cs = 0xAA;
    for (uint8_t b : buf)
      cs -= b;
    if (cs == 0)
      send(buf);
    buf.clear();
  }
}

class StreamServerComponent : public esphome::Component {
 public:
  StreamServerComponent() = default;
  explicit StreamServerComponent(UARTComponent *uart) : uart_{uart} {}
  void set_uart_parent(UARTComponent *uart) { this->uart_ = uart; }

  float get_setup_priority() const override { return esphome::setup_priority::AFTER_WIFI; }
  void setup() override;
  void loop() override;
  void dump_config() override;
  void on_shutdown() override;

  void bind(uint16_t port);
  void on_uart_data(OnUartDataCb cb) { this->on_uart_data_cb_ = cb; }

 protected:
  void cleanup();
  void read();

  AsyncServer server_{0};
  UARTComponent *uart_{nullptr};
  OnUartDataCb on_uart_data_cb_{nullptr};
  std::vector<AsyncClient *> clients_{};
};
