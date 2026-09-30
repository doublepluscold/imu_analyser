#pragma once
// MAC -> module_id table. IDs are handed out in order of first appearance: 1, 2, 3...
// No persistence in here; the caller saves/loads entries (NVS on the master).
#include <stddef.h>
#include <stdint.h>

class IdTable {
public:
    static constexpr size_t kMaxModules = 8;

    struct Entry {
        uint8_t mac[6];
        uint8_t id;  // 1..kMaxModules
    };

    // Returns the id for mac, assigning the next free one if it is new.
    // Returns 0 if the table is full and mac is unknown.
    uint8_t lookup_or_assign(const uint8_t mac[6]);
    // Returns the id or 0 if unknown. Never assigns.
    uint8_t find(const uint8_t mac[6]) const;

    size_t count() const { return count_; }
    const Entry &entry(size_t i) const { return entries_[i]; }
    // True if the last lookup_or_assign() created a new entry (caller should save).
    bool dirty() const { return dirty_; }
    void clear_dirty() { dirty_ = false; }

    // Restores saved state. Rejects (returns false, table left empty) more than
    // kMaxModules entries, id 0 or > kMaxModules, duplicate ids or duplicate MACs.
    bool load(const Entry *entries, size_t n);
    void reset();

private:
    Entry entries_[kMaxModules];
    size_t count_ = 0;
    bool dirty_ = false;
};
