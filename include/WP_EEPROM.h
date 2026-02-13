#pragma once

#include <Arduino.h>
#include <EEPROM.h>
#include <Locations.h>

// External global waypoint data
extern Locations waypoint[100];
extern int wp_sum;

// EEPROM addresses
namespace WP_EEPROM {
    const uint16_t MAGIC_NUMBER = 0xABCD;  // Validation marker
    const uint16_t ADDR_MAGIC = 0;         // Magic number address
    const uint16_t ADDR_WP_COUNT = 2;      // Waypoint count address
    const uint16_t ADDR_CHECKSUM = 4;      // Checksum address
    const uint16_t ADDR_WP_DATA = 8;       // Waypoint data start address
    const uint16_t MAX_EEPROM_WP = 50;     // Max waypoints in EEPROM (limited by size)
}

// Calculate simple checksum for validation
uint16_t calculateWaypointChecksum() {
    uint16_t checksum = 0;
    for (int i = 0; i < wp_sum; i++) {
        checksum ^= (waypoint[i].lat >> 16) & 0xFFFF;
        checksum ^= (waypoint[i].lat) & 0xFFFF;
        checksum ^= (waypoint[i].lng >> 16) & 0xFFFF;
        checksum ^= (waypoint[i].lng) & 0xFFFF;
        checksum ^= (waypoint[i].alt >> 16) & 0xFFFF;
        checksum ^= (waypoint[i].alt) & 0xFFFF;
    }
    return checksum;
}

// Save waypoints to EEPROM
bool saveWaypointsToEEPROM() {
    if (wp_sum > WP_EEPROM::MAX_EEPROM_WP) {
        Serial.printf("ERROR: Too many waypoints to save (%d > %d)\n", 
                      wp_sum, WP_EEPROM::MAX_EEPROM_WP);
        return false;
    }

    Serial.printf("Saving %d waypoints to EEPROM...\n", wp_sum);
    
    // Write magic number
    EEPROM.put(WP_EEPROM::ADDR_MAGIC, WP_EEPROM::MAGIC_NUMBER);
    
    // Write waypoint count
    EEPROM.put(WP_EEPROM::ADDR_WP_COUNT, (uint16_t)wp_sum);
    
    // Calculate and write checksum
    uint16_t checksum = calculateWaypointChecksum();
    EEPROM.put(WP_EEPROM::ADDR_CHECKSUM, checksum);
    
    // Write waypoint data
    for (int i = 0; i < wp_sum; i++) {
        uint16_t addr = WP_EEPROM::ADDR_WP_DATA + (i * sizeof(Locations));
        EEPROM.put(addr, waypoint[i]);
    }
    
    Serial.printf("✓ Waypoints saved to EEPROM (checksum: 0x%04X)\n", checksum);
    return true;
}

// Load waypoints from EEPROM
bool loadWaypointsFromEEPROM() {
    // Check magic number
    uint16_t magic;
    EEPROM.get(WP_EEPROM::ADDR_MAGIC, magic);
    
    if (magic != WP_EEPROM::MAGIC_NUMBER) {
        Serial.println("No valid waypoint data in EEPROM (magic mismatch)");
        return false;
    }
    
    // Read waypoint count
    uint16_t saved_count;
    EEPROM.get(WP_EEPROM::ADDR_WP_COUNT, saved_count);
    
    if (saved_count > WP_EEPROM::MAX_EEPROM_WP) {
        Serial.printf("ERROR: Invalid waypoint count in EEPROM (%d)\n", saved_count);
        return false;
    }
    
    // Read waypoint data temporarily
    Locations temp_waypoint[100];
    for (int i = 0; i < saved_count; i++) {
        uint16_t addr = WP_EEPROM::ADDR_WP_DATA + (i * sizeof(Locations));
        EEPROM.get(addr, temp_waypoint[i]);
    }
    
    // Temporarily set wp_sum for checksum calculation
    int temp_wp_sum = wp_sum;
    wp_sum = saved_count;
    
    // Copy to waypoint array for checksum
    for (int i = 0; i < saved_count; i++) {
        waypoint[i] = temp_waypoint[i];
    }
    
    // Verify checksum
    uint16_t calculated_checksum = calculateWaypointChecksum();
    uint16_t saved_checksum;
    EEPROM.get(WP_EEPROM::ADDR_CHECKSUM, saved_checksum);
    
    if (calculated_checksum != saved_checksum) {
        Serial.printf("ERROR: Checksum mismatch (calc: 0x%04X, saved: 0x%04X)\n", 
                      calculated_checksum, saved_checksum);
        wp_sum = temp_wp_sum;  // Restore original count
        return false;
    }
    
    // Success - keep the loaded data
    wp_sum = saved_count;
    
    Serial.printf("✓ Loaded %d waypoints from EEPROM\n", wp_sum);
    for (int i = 0; i < wp_sum && i < 3; i++) {  // Print first 3
        Serial.printf("  WP[%d]: lat=%d, lng=%d, alt=%dcm\n", 
                      i, waypoint[i].lat, waypoint[i].lng, waypoint[i].alt);
    }
    if (wp_sum > 3) {
        Serial.printf("  ... and %d more\n", wp_sum - 3);
    }
    
    return true;
}

// Clear EEPROM waypoint data
void clearWaypointEEPROM() {
    EEPROM.put(WP_EEPROM::ADDR_MAGIC, (uint16_t)0x0000);
    EEPROM.put(WP_EEPROM::ADDR_WP_COUNT, (uint16_t)0);
    Serial.println("✓ EEPROM waypoint data cleared");
}

// Check if valid waypoints exist in EEPROM
bool hasValidWaypointsInEEPROM() {
    uint16_t magic;
    EEPROM.get(WP_EEPROM::ADDR_MAGIC, magic);
    return (magic == WP_EEPROM::MAGIC_NUMBER);
}

// Print EEPROM status
void printEEPROMStatus() {
    if (hasValidWaypointsInEEPROM()) {
        uint16_t saved_count;
        EEPROM.get(WP_EEPROM::ADDR_WP_COUNT, saved_count);
        Serial.printf("EEPROM: %d waypoints stored\n", saved_count);
    } else {
        Serial.println("EEPROM: No waypoints stored");
    }
}
