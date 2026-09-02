#include "neato_lidar.h"
#include "esphome/core/log.h"

namespace esphome {
namespace neato_lidar {

static const char *TAG = "neato_lidar";

void NeatoLidarComponent::setup() {
  ESP_LOGCONFIG(TAG, "NeatoLidarComponent: device_id='%s', topic='%s', interval=%u ms",
                device_id_.c_str(), mqtt_topic_.c_str(), publish_interval_);
  ESP_LOGCONFIG(TAG, "Ring buffer capacity: %u bytes (scan size: %u bytes)",
                LIDAR_RING_SIZE, LIDAR_SCAN_SIZE);
}

void NeatoLidarComponent::on_uart_data(uint8_t data) {
  // On écrit chaque octet dans le ring buffer
  if (!ring_.write_byte(data)) {
    // Buffer plein : on réinitialise pour éviter la corruption
    ESP_LOGW(TAG, "Ring buffer full, resetting parse state");
    ring_.clear();
    scan_in_progress_ = false;
    bytes_received_ = 0;
    return;
  }

  bytes_received_++;

  // On tente de parser dès qu'on a assez d'octets
  if (bytes_received_ >= LIDAR_SCAN_SIZE) {
    if (try_parse_and_publish()) {
      bytes_received_ = 0;
      scan_in_progress_ = false;
    }
  }
}

bool NeatoLidarComponent::try_parse_and_publish() {
  // On a besoin d'au moins LIDAR_SCAN_SIZE octets dans le ring
  if (ring_.size() < LIDAR_SCAN_SIZE) {
    return false;
  }

  // On copie le scan dans un buffer statique (évite l'alloc dynamique)
  // 4 (header) + 5400 (scan) = 5404 octets
  static uint8_t msg_buf[LIDAR_SCAN_SIZE + 4];

  // Header
  msg_buf[0] = LIDAR_MAGIC_0;
  msg_buf[1] = LIDAR_MAGIC_1;
  msg_buf[2] = 0x01;   // 1 scan complet
  msg_buf[3] = LIDAR_FORMAT_VERSION;

  // Copie du scan depuis le ring buffer
  size_t copied = ring_.copy_to(msg_buf + 4, LIDAR_SCAN_SIZE);
  if (copied < LIDAR_SCAN_SIZE) {
    ESP_LOGW(TAG, "Incomplete scan: got %u / %u bytes", copied, LIDAR_SCAN_SIZE);
    return false;
  }

  // Validation rapide : vérifier que les angles sont cohérents
  // (angle[i] devrait être proche de i, modulo 1080)
  bool valid = true;
  for (size_t i = 0; i < LIDAR_POINTS_PER_SCAN; i++) {
    size_t offset = 4 + i * LIDAR_BYTES_PER_POINT;
    uint16_t angle = static_cast<uint16_t>(msg_buf[offset]) |
                      (static_cast<uint16_t>(msg_buf[offset + 1]) << 8);
    // Tolérance de ±5 points (1.65°)
    int16_t diff = static_cast<int16_t>(angle - static_cast<int16_t>(i));
    if (diff > 5 || diff < -5) {
      // On ajuste : le scan peut être décalé
      // On ne rejette pas, on log juste
      if (i == 0) {
        ESP_LOGD(TAG, "Scan offset detected: first angle=%u (expected 0)", angle);
      }
      break;
    }
  }

  // Publication MQTT
  publish_scan(msg_buf, LIDAR_SCAN_SIZE + 4);
  return true;
}

void NeatoLidarComponent::publish_scan(const uint8_t *data, size_t len) {
  // On publie en binaire (retain=false, qos=0 pour la bande passante)
  auto payload = mqtt::MQTTMessagePayload(data, len);
  this->publish_global(mqtt_topic_, payload, 0, false);

  last_publish_ms_ = millis();
  ESP_LOGD(TAG, "Published scan: %u bytes to '%s'", len, mqtt_topic_.c_str());
}

void NeatoLidarComponent::update() {
  // La publication se fait dans on_uart_data() dès qu'un scan complet est reçu.
  // update() sert juste à envoyer la commande périodiquement si le robot
  // ne l'envoie pas en continu.
  // On n'envoie pas GetLDSScan ici car le robot Neato stream en continu
  // quand le mode lidar est actif (contrôlé par les entités existantes).
  // On peut juste log le taux de publication.
  static uint32_t last_log_ms = 0;
  if (millis() - last_log_ms > 10000) {
    ESP_LOGD(TAG, "Lidar stream active, last publish %u ms ago",
             millis() - last_publish_ms_);
    last_log_ms = millis();
  }
}

}   // namespace neato_lidar
}   // namespace esphome
