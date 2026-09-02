#pragma once

#include <cstdint>
#include <cstddef>
#include <cstring>

/**
 * RingBuffer<T, N>
 * Buffer circulaire de taille fixe, alloué statiquement.
 * Conçu pour l'ESP32 classique (520 KB SRAM) :
 *      - Pas de malloc / new
 *      - Pas de fragmentation
 *      - Taille connue au compile-time
 *
 * Usage typique :
 *   RingBuffer<uint8_t, 4096> scan_buf;
 *   scan_buf.write(ptr, len);
 *   scan_buf.read(out, len);
 */
template <typename T, std::size_t N>
class RingBuffer {
 public:
  RingBuffer() noexcept : head_(0u), tail_(0u), count_(0u) {}

  // --- Écriture ---
  // Renvoie le nombre d'octets réellement écrits (0 si plein).
  std::size_t write(const T *data, std::size_t len) noexcept {
    if (len == 0) return 0;
    std::size_t space = N - count_;
    std::size_t to_write = (len < space) ? len : space;
    for (std::size_t i = 0; i < to_write; i++) {
      buffer_[head_] = data[i];
      head_ = (head_ + 1) % N;
    }
    count_ += to_write;
    return to_write;
  }

  // Écrire un seul octet (cas fréquent dans le parsing UART)
  bool write_byte(uint8_t byte) noexcept {
    if (count_ == N) return false;
    buffer_[head_] = byte;
    head_ = (head_ + 1) % N;
    count_++;
    return true;
  }

  // --- Lecture ---
  // Renvoie le nombre d'octets réellement lus.
  std::size_t read(T *out, std::size_t len) noexcept {
    if (len == 0) return 0;
    std::size_t to_read = (len < count_) ? len : count_;
    for (std::size_t i = 0; i < to_read; i++) {
      out[i] = buffer_[tail_];
      tail_ = (tail_ + 1) % N;
    }
    count_ -= to_read;
    return to_read;
  }

  // Lire un seul octet sans dépiler (peek).
  // Renvoie false si le buffer est vide (évite de retourner une valeur par défaut trompeuse).
  bool peek(T &out) const noexcept {
    if (count_ == 0) return false;
    out = buffer_[tail_];
    return true;
  }

  // --- État ---
  bool empty() const noexcept { return count_ == 0; }
  bool full() const noexcept { return count_ == N; }
  std::size_t size() const noexcept { return count_; }
  std::size_t capacity() const noexcept { return N; }

  // Vider le buffer
  void clear() noexcept {
    head_ = 0u;
    tail_ = 0u;
    count_ = 0u;
  }

  // Copier tout le contenu dans un buffer externe (sans dépiler)
  // Renvoie le nombre d'octets copiés.
  std::size_t copy_to(T *out, std::size_t max_len) const noexcept {
    std::size_t to_copy = (count_ < max_len) ? count_ : max_len;
    for (std::size_t i = 0; i < to_copy; i++) {
      out[i] = buffer_[(tail_ + i) % N];
    }
    return to_copy;
  }

 private:
  T buffer_[N];
  std::size_t head_;
  std::size_t tail_;
  std::size_t count_;
};
