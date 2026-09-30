#include "id_table.h"

#include <string.h>

uint8_t IdTable::find(const uint8_t mac[6]) const {
    for (size_t i = 0; i < count_; i++)
        if (memcmp(entries_[i].mac, mac, 6) == 0) return entries_[i].id;
    return 0;
}

uint8_t IdTable::lookup_or_assign(const uint8_t mac[6]) {
    uint8_t id = find(mac);
    if (id) return id;
    if (count_ >= kMaxModules) return 0;

    // Smallest unused id.
    for (id = 1; id <= kMaxModules; id++) {
        bool used = false;
        for (size_t i = 0; i < count_; i++)
            if (entries_[i].id == id) used = true;
        if (!used) break;
    }
    memcpy(entries_[count_].mac, mac, 6);
    entries_[count_].id = id;
    count_++;
    dirty_ = true;
    return id;
}

bool IdTable::load(const Entry *entries, size_t n) {
    reset();
    if (n > kMaxModules) return false;
    for (size_t i = 0; i < n; i++) {
        if (entries[i].id == 0 || entries[i].id > kMaxModules) return false;
        for (size_t j = 0; j < i; j++) {
            if (entries[j].id == entries[i].id) return false;
            if (memcmp(entries[j].mac, entries[i].mac, 6) == 0) return false;
        }
    }
    for (size_t i = 0; i < n; i++) entries_[i] = entries[i];
    count_ = n;
    return true;
}

void IdTable::reset() {
    count_ = 0;
    dirty_ = false;
}
