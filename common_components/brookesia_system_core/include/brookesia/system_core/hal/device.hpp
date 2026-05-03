/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include <list>
#include <map>
#include <string>
#include <string_view>
#include <unordered_map>
#include <array>
#include <memory>
#include "brookesia/system_core/macro_configs.h"
#include "esp_lib_utils.h"

namespace esp_brookesia::systems::hal {

//---------------- Interface ID ----------------
static inline constexpr uint64_t fnv1a_64(std::string_view s)
{
    uint64_t hash = 0xcbf29ce484222325;
    for (char c : s) {
        hash ^= static_cast<uint8_t>(c);
        hash *= 0x100000001b3;
    }
    return hash;
}
template<typename I> constexpr uint64_t IID()
{
    return fnv1a_64(I::interface_name);
}

/**
 * @brief The core app class. This serves as the base class for all internal app classes. User-defined app classes
 *        should not inherit from this class directly.
 *
 */
class Device {
public:

    using Registry = esp_utils::PluginRegistry<Device>;

    /**
     * @brief Delete copy constructor and assignment operator
     */
    Device(const Device &) = delete;
    Device(Device &&) = delete;
    Device &operator=(const Device &) = delete;
    Device &operator=(Device &&) = delete;

    /**
     * @brief Construct a core app with detailed configuration.
     *
     * @param data The configuration data for the core app.
     *
     */
    Device(const char *name)
        : _name(name)
    {
    }

    /**
     * @brief Destructor for the core app, should be defined by the user's app class.
     */
    virtual ~Device() = default;

    virtual void *QueryInterface(uint64_t)
    {
        return nullptr;
    }
    template<typename I> I *Query()
    {
        return static_cast<I *>(QueryInterface(IID<I>()));
    }

    virtual bool probe()
    {
        return false;
    }

    /**
     * @brief  Check if the app is initialized
     *
     * @return true if the app is initialized, otherwise false
     *
     */
    virtual bool checkInitialized(void) const
    {
        return false;
    }

    /**
     * @brief Get the app name
     *
     * @return name: the string pointer of the app name
     *
     */
    const std::string &getName(void) const
    {
        return _name;
    }

    const void *getHandle() const
    {
        return _dev_handle;
    }

    /**
     * @brief Called when the app starts to install. The app can perform initialization here.
     *
     * @return true if successful, otherwise false
     *
     */
    virtual bool init(void)
    {
        return true;
    }

    /**
     * @brief Called when the app starts to uninstall. The app can perform deinitialization here.
     *
     * @return true if successful, otherwise false
     *
     */
    virtual bool deinit(void)
    {
        return true;
    }

    /**
     * @brief Initialize all devices from the registry
     *
     * @return always true
     *
     */
    static bool initDeviceFromRegistry(void)
    {
        Device::Registry::forEach([&](const auto & plugin) {
            ESP_UTILS_TAG_LOGI(
                BROOKESIA_SYSTEM_CORE_LOG_TAG, "Found Device plugin: %s", plugin.name.c_str()
            );
            std::shared_ptr<Device> dev = Device::Registry::get(plugin.name);
            if (dev == nullptr) {
                ESP_UTILS_TAG_LOGE(
                    BROOKESIA_SYSTEM_CORE_LOG_TAG, "\t - Get instance failed"
                );
                return;
            }
            if (!dev->probe()) {
                ESP_UTILS_TAG_LOGE(
                    BROOKESIA_SYSTEM_CORE_LOG_TAG, "\t - Probe failed"
                );
                return;
            }
            if (!dev->init()) {
                ESP_UTILS_TAG_LOGE(
                    BROOKESIA_SYSTEM_CORE_LOG_TAG, "\t - Init failed"
                );
                return;
            }
            ESP_UTILS_TAG_LOGI(
                BROOKESIA_SYSTEM_CORE_LOG_TAG, "\t - Init success"
            );
        });

        return true;
    }

protected:
    void *_dev_handle = nullptr;   /**< The handle of the device */

private:
    std::string _name = "BaseDevice"; /**< The name of the app */
};

//----------------  CRTP Implementation ----------------
template<typename Derived>
class DeviceImpl : public Device {

protected:
    DeviceImpl(const char *name): Device(name) {}
    template<typename... Ifaces>
    void *BuildTable(uint64_t id)
    {
        static const Entry table[] = { Make<Ifaces>()... };
        for (const auto &e : table) if (e.id == id) {
                return e.cast(static_cast<Derived *>(this));
            }
        return nullptr;
    }

private:
    struct Entry {
        uint64_t id;
        void *(*cast)(Derived *);
    };
    template<typename I> static Entry Make()
    {
        return {IID<I>(), [](Derived * self)->void *{ return static_cast<I *>(self); }};
    }
};

} // namespace esp_brookesia::systems::base
