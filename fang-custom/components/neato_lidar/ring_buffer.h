#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>

/**
 * RingBuffer<T, N>
 * Buffer circulaire de taille fixe, alloué statiquement.
 * Conçu pour l'ESP32 classique (520 KB SRAM) :
 *    - Pas de malloc / new
 *    - Pas de fragmentation
 *    - Taille connue au compile-time
 *
 * Usage typique :
 *   RingBuffer<uint8_t, 4096> scan_buf;
 *   scan_buf.write(ptr, len);
 *   scan_buf.read(out, len);
 */
template <typename T, size_t N>
class RingBuffer {
 public:
  RingBuffer() : head_(0), tail_(0), count_(0) {}

  // --- Écriture ---
  // Renvoie le nombre d'octets réellement écrits (0 si plein).
  size_t write(const T *data, size_t len) {
    if (len == 0) return 0;
    size_t space = N - count_;
    size_t to_write = (len < space) ? len : space;
    for (size_t i = 0; i < to_write; i++) {
      buffer_[head_] = data[i];
      head_ = (head_ + 1) % N;
    }
    count_ += to_write;
    return to_write;
  }

  // Écrire un seul octet (cas fréquent dans le parsing UART)
  bool write_byte(uint8_t byte) {
    if (count_ == N) return false;
    buffer_[head_] = byte;
    head_ = (head_ + 1) % N;
    count_++;
    return true;
  }

  // --- Lecture ---
  // Renvoie le nombre d'octets réellement lus.
  size_t read(T *out, size_t len) {
    if (len == 0) return 0;
    size_t to_read = (len < count_) ? len : count_;
    for (size_t i = 0; i < to_read; i++) {
      out[i] = buffer_[tail_];
      tail_ = (tail_ + 1) % N;
    }
    count_ -= to_read;
    return to_read;
  }

  // Lire un seul octet sans dépiler (peek)
  T peek() const {
    return (count_ > 0) ? buffer_[tail_] : T{};
  }

  // --- État ---
  bool empty() const { return count_ == 0; }
  bool full()  const { return count_ == N; }
  size_t size() const { return count_; }
  size_t capacity() const { return N; }

  // Vider le buffer
  void clear() {
    head_ = 0;
    tail_ = 0;
    count_ = 0;
  }

  // Copier tout le contenu dans un buffer externe (sans dépiler)
  // Renvoie le nombre d'octets copiés.
  size_t copy_to(T *out, size_t max_len) const {
    size_t to_copy = (count_ < max_len) ? count_ : max_len;
    for (size_t i = 0; i < to_copy; i++) {
      out[i] = buffer_[(tail_ + i) % N];
    }
    return to_copy;
  }

 private:
  T buffer_[N];
  size_t head_;
  size_t tail_;
  size_t count_;
};
