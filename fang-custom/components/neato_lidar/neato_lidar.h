#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/mqtt/mqtt_component.h"
#include "ring_buffer.h"

namespace esphome {
namespace neato_lidar {

// --- Constantes du protocole lidar Neato ---
// Un scan complet = 1080 points (résolution 0.33°)
// Chaque point : angle (uint16 LE, 0..1079), distance (uint16 LE, mm), intensité (uint8)
// Taille d'un point = 5 octets
// Taille d'un scan complet = 1080 * 5 = 5400 octets
// On réserve 6144 pour la marge (header + padding)
inline constexpr std::size_t LIDAR_POINTS_PER_SCAN = 1080;
inline constexpr std::size_t LIDAR_BYTES_PER_POINT = 5;
inline constexpr std::size_t LIDAR_SCAN_SIZE = LIDAR_POINTS_PER_SCAN * LIDAR_BYTES_PER_POINT;
inline constexpr std::size_t LIDAR_RING_SIZE = 6144;  // > 5400, marge pour header

// Header du message MQTT binaire (4 octets) :
//   byte 0-1 : magic 0x4C 0x44 ("LD")
//   byte 2     : nombre de points / 1080 (1 = scan complet)
//   byte 3     : version du format (0x01)
inline constexpr uint8_t LIDAR_MAGIC_0 = 0x4C;
inline constexpr uint8_t LIDAR_MAGIC_1 = 0x44;
inline constexpr uint8_t LIDAR_FORMAT_VERSION = 0x01;

// Commande UART pour demander un scan lidar
inline constexpr const char *CMD_GET_LDS_SCAN = "GetLDSScan\n";

// Héritage :
//   PollingComponent  -> Component (virtual)
//   MQTTComponent     -> Component (virtual)
//   UARTDevice        -> pas d'héritage de Component (simple holder de uart_component_)
//   UARTReceiver      -> interface pure (on_data)
// Le virtual sur PollingComponent et MQTTComponent résout le diamond sur Component.
class NeatoLidarComponent : public virtual PollingComponent,
                           public uart::UARTDevice,
                           public uart::UARTReceiver,
                           public virtual mqtt::MQTTComponent {
 public:
  // Ne pas passer d'intervalle au constructeur : il sera configuré via set_scan_interval()
  NeatoLidarComponent() = default;

  void set_device_id(const std::string &id) { device_id_ = id; }
  void set_scan_interval(uint32_t ms) { this->publish_interval_ = ms; }
  void set_mqtt_topic(const std::string &topic) { mqtt_topic_ = topic; }

  // PollingComponent
  void update() override;

  // UARTReceiver : appelé par le composant UART à chaque octet reçu
  void on_data(uint8_t data) override;

  // Component / MQTTComponent
  void setup() override;

  float get_setup_priority() const override { return setup_priority::DATA; }

 protected:
  // Parsing d'une trame complète dans le ring buffer
  // Renvoie true si un scan complet a été assemblé et publié
  bool try_parse_and_publish();

  // Publier le scan binaire sur MQTT
  void publish_scan(const uint8_t *data, std::size_t len);

  // --- Membres ---
  std::string device_id_;
  std::string mqtt_topic_;
  uint32_t publish_interval_ = 100;  // ms

  // Ring buffer statique : pas d'allocation dynamique
  RingBuffer<uint8_t, LIDAR_RING_SIZE> ring_;

  // État du parsing
  bool scan_in_progress_ = false;
  std::size_t bytes_received_ = 0;
  uint32_t last_publish_ms_ = 0;
};

}  // namespace neato_lidar
}  // namespace esphome
