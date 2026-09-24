#include "zy100_mode_workspace.h"

#include <stddef.h>
#include <string.h>

#include "os_sync.h"

static uint32_t s_mode_workspace_words[ZY100_MODE_WORKSPACE_BYTES /
                                       sizeof(uint32_t)];
static zy100_mode_workspace_owner_t s_mode_workspace_owner =
    ZY100_MODE_WORKSPACE_OWNER_NONE;

static uint32_t s_online_token;
static uint8_t s_online_clients;

typedef char zy100_online_workspace_layout_check[
    ((ZY100_ONLINE_WORKSPACE_IMU_BYTES + ZY100_ONLINE_WORKSPACE_MAG_BYTES ==
      ZY100_ONLINE_WORKSPACE_PAGE_OFFSET) &&
     (ZY100_ONLINE_WORKSPACE_PAGE_OFFSET + 256U == ZY100_ONLINE_WORKSPACE_VERIFY_OFFSET) &&
     (ZY100_ONLINE_WORKSPACE_VERIFY_OFFSET + 256U == ZY100_ONLINE_WORKSPACE_TX_OFFSET) &&
     (ZY100_ONLINE_WORKSPACE_TX_OFFSET + ZY100_ONLINE_WORKSPACE_TX_BYTES *
      ZY100_ONLINE_WORKSPACE_TX_SLOTS == ZY100_ONLINE_WORKSPACE_REQUIRED_BYTES) &&
     (ZY100_ONLINE_WORKSPACE_REQUIRED_BYTES <= ZY100_MODE_WORKSPACE_TRANSIENT_BYTES) &&
     ((ZY100_ONLINE_WORKSPACE_PAGE_OFFSET | ZY100_ONLINE_WORKSPACE_VERIFY_OFFSET |
       ZY100_ONLINE_WORKSPACE_TX_OFFSET) % sizeof(uint32_t) == 0U)) ? 1 : -1];

typedef char zy100_mode_workspace_size_check[
    (sizeof(s_mode_workspace_words) == ZY100_MODE_WORKSPACE_BYTES) ? 1 : -1];

bool zy100_mode_workspace_claim(zy100_mode_workspace_owner_t owner,
                                uint32_t required_bytes,
                                uint8_t **base_out,
                                uint32_t *bytes_out)
{
    uint32_t lock_state;

    if ((owner == ZY100_MODE_WORKSPACE_OWNER_NONE) ||
        (required_bytes == 0U) ||
        (required_bytes > ZY100_MODE_WORKSPACE_TRANSIENT_BYTES) ||
        (base_out == NULL) || (bytes_out == NULL))
    {
        return false;
    }

    lock_state = os_lock();
    if (s_mode_workspace_owner != ZY100_MODE_WORKSPACE_OWNER_NONE)
    {
        os_unlock(lock_state);
        return false;
    }
    s_mode_workspace_owner = owner;
    os_unlock(lock_state);

    memset(s_mode_workspace_words, 0, ZY100_MODE_WORKSPACE_TRANSIENT_BYTES);
    *base_out = (uint8_t *)&s_mode_workspace_words[0];
    *bytes_out = ZY100_MODE_WORKSPACE_TRANSIENT_BYTES;
    return true;
}

bool zy100_mode_workspace_release(zy100_mode_workspace_owner_t owner)
{
    uint32_t lock_state;

    if (owner == ZY100_MODE_WORKSPACE_OWNER_NONE)
    {
        return false;
    }

    lock_state = os_lock();
    if ((s_mode_workspace_owner != owner) || (s_online_clients != 0U))
    {
        os_unlock(lock_state);
        return false;
    }
    s_mode_workspace_owner = ZY100_MODE_WORKSPACE_OWNER_NONE;
    os_unlock(lock_state);
    return true;
}

zy100_mode_workspace_owner_t zy100_mode_workspace_owner(void)
{
    zy100_mode_workspace_owner_t owner;
    uint32_t lock_state = os_lock();

    owner = s_mode_workspace_owner;
    os_unlock(lock_state);
    return owner;
}

bool zy100_mode_workspace_available(void)
{
    return zy100_mode_workspace_owner() == ZY100_MODE_WORKSPACE_OWNER_NONE;
}

uint8_t *zy100_mode_workspace_persistent_base(uint32_t *bytes_out)
{
    if (bytes_out != NULL)
    {
        *bytes_out = ZY100_MODE_WORKSPACE_PERSISTENT_BYTES;
    }
    return ((uint8_t *)&s_mode_workspace_words[0]) +
           ZY100_MODE_WORKSPACE_TRANSIENT_BYTES;
}

bool zy100_mode_workspace_online_open(uint32_t *token, uint8_t **base)
{
    uint32_t bytes;
    uint32_t lock_state;
    if ((token == NULL) || (base == NULL) ||
        !zy100_mode_workspace_claim(ZY100_MODE_WORKSPACE_OWNER_ONLINE,
            ZY100_ONLINE_WORKSPACE_REQUIRED_BYTES, base, &bytes))
    {
        return false;
    }
    lock_state = os_lock();
    s_online_token++;
    if (s_online_token == 0U) { s_online_token++; }
    s_online_clients = ZY100_ONLINE_WORKSPACE_TX;
    *token = s_online_token;
    os_unlock(lock_state);
    return true;
}

bool zy100_mode_workspace_online_join(uint32_t *token, uint8_t **base)
{
    uint32_t lock_state;
    if ((token == NULL) || (base == NULL)) { return false; }
    lock_state = os_lock();
    if ((s_mode_workspace_owner != ZY100_MODE_WORKSPACE_OWNER_ONLINE) ||
        (s_online_clients != ZY100_ONLINE_WORKSPACE_TX))
    {
        os_unlock(lock_state);
        return false;
    }
    s_online_clients |= ZY100_ONLINE_WORKSPACE_CAPTURE;
    *token = s_online_token;
    *base = (uint8_t *)s_mode_workspace_words;
    os_unlock(lock_state);
    return true;
}

bool zy100_mode_workspace_online_release(uint32_t token,
                                         zy100_online_workspace_client_t client)
{
    uint32_t lock_state = os_lock();
    if ((s_mode_workspace_owner != ZY100_MODE_WORKSPACE_OWNER_ONLINE) ||
        (token == 0U) || (token != s_online_token) ||
        ((client != ZY100_ONLINE_WORKSPACE_TX) &&
         (client != ZY100_ONLINE_WORKSPACE_CAPTURE)) ||
        ((s_online_clients & (uint8_t)client) == 0U))
    {
        os_unlock(lock_state);
        return false;
    }
    s_online_clients &= (uint8_t)~client;
    if (s_online_clients == 0U)
    {
        s_mode_workspace_owner = ZY100_MODE_WORKSPACE_OWNER_NONE;
    }
    os_unlock(lock_state);
    return true;
}
