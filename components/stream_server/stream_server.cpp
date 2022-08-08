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

#include "stream_server.h"

#include "esphome/core/log.h"
#include "esphome/core/util.h"
#include "esphome/components/network/util.h"

static const char *TAG = "streamserver";

using namespace esphome;

void StreamServerComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up stream server...");
  if (this->on_uart_data_cb_ == nullptr)
    this->on_uart_data([](const std::vector<uint8_t> &in, SendCb send) { send(in); });
  this->server_.begin();
  this->server_.setNoDelay(true);
  this->server_.onClient(
      [this](void *arg, AsyncClient *client) {
        ESP_LOGD(TAG, "Client %s connected", client->remoteIP().toString().c_str());
        this->clients_.push_back(client);
        client->onData(
            [this](void *, AsyncClient *, void *data, size_t len) {
              if (data != nullptr && len > 0)
                this->uart_->write_array(reinterpret_cast<uint8_t *>(data), len);
            },
            nullptr);
      },
      nullptr);
}

void StreamServerComponent::loop() {
  this->cleanup();
  this->read();
}

void StreamServerComponent::on_shutdown() {
  for (auto client : this->clients_)
    client->close(true);
}

void StreamServerComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "Stream Server:");
  ESP_LOGCONFIG(TAG, "  Address: %s:%u", esphome::network::get_ip_address().str().c_str(), this->port_);
}

void StreamServerComponent::bind(uint16_t port) { this->server_ = AsyncServer(port); }

void StreamServerComponent::cleanup() {
  auto end = this->clients_.end();
  auto last = std::partition(this->clients_.begin(), end, [](AsyncClient *c) { return c->disconnected() == false; });
  for (auto it = last; it != end; ++it) {
    ESP_LOGD(TAG, "Client %s disconnected", (*it)->remoteIP().toString().c_str());
    delete *it;
  }
  this->clients_.erase(last, end);
}

void StreamServerComponent::read() {
  int len = this->uart_->available();
  if (len > 0) {
    std::vector<uint8_t> buf(len);
    this->uart_->read_array(buf.data(), buf.size());
    this->on_uart_data_cb_(buf, [this](const std::vector<uint8_t> &buf) {
      for (auto client : this->clients_)
        client->write(reinterpret_cast<const char *>(buf.data()), buf.size());
    });
  }
}
