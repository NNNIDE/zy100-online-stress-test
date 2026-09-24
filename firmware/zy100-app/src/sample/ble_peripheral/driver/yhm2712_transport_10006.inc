/* Private transport implementation: frozen 10006, selected by hw_config. */
#if YHM2712_ACMD_TX_CLOSE_DATA_LOW_PULSE == 1U
YHM2712_ACMD_FORCE_INLINE void yhm2712_acmd_drive_data_low_width_closed(
    uint32_t low_ticks)
{
    uint32_t start;

    /*
     * Factory edge-timed B0/B1 experiment.  Direction/source ownership and
     * every variable-time preparation step are completed while the line is
     * high.  Start the deadline only after DATAOUT creates the falling edge:
     * a first-access delay before that edge must not shorten a B0/B1 pulse.
     * The low phase otherwise contains only the deadline wait and matching
     * DATAOUT rising edge.
     */
    GPIO->DATAOUT = s_yhm2712_acmd.fast_gpio.dataout_low;
    start = yhm2712_acmd_tick_now();
    while (yhm2712_acmd_tick_delta(start, yhm2712_acmd_tick_now()) < low_ticks)
    {
    }
    GPIO->DATAOUT = s_yhm2712_acmd.fast_gpio.dataout_high;
}
#endif

static yhm2712_acmd_status_t yhm2712_acmd_wait_released_high(uint32_t ticks)
{
    const uint32_t start = yhm2712_acmd_tick_now();

    yhm2712_acmd_release_fast();
    while (yhm2712_acmd_tick_delta(start, yhm2712_acmd_tick_now()) < ticks)
    {
        if (!yhm2712_acmd_line_is_high())
        {
            return YHM2712_ACMD_STATUS_LINE_BUSY_LOW;
        }
    }
    return YHM2712_ACMD_STATUS_OK;
}

bool yhm2712_acmd_converge_status_input(
    yhm2712_acmd_recovery_snapshot_t *snapshot)
{
    uint32_t primask_before = __get_PRIMASK();

    yhm2712_acmd_recovery_snapshot_clear();
    if (!s_yhm2712_acmd.inited || (s_yhm2712_acmd.gpio_pin == 0U) ||
        s_yhm2712_acmd.busy)
    {
        yhm2712_acmd_recovery_snapshot_record(primask_before, false);
    }
    else
    {
        yhm2712_acmd_restore_status_input();
        yhm2712_acmd_recovery_snapshot_record(primask_before, true);
    }

    if (snapshot != NULL)
    {
        *snapshot = s_yhm2712_acmd_last_recovery;
    }

    return s_yhm2712_acmd_last_recovery.valid &&
           s_yhm2712_acmd_last_recovery.status_input_restored &&
           s_yhm2712_acmd_last_recovery.primask_restored &&
           s_yhm2712_acmd_last_recovery.busy_cleared;
}

#if YHM2712_ACMD_FACTORY_RAM_TIMING_ENABLE == 1U
YHM2712_ACMD_FORCE_INLINE uint8_t yhm2712_acmd_parity_bit(uint8_t value)
#else
static uint8_t yhm2712_acmd_parity_bit(uint8_t value)
#endif
{
    uint8_t parity = 0U;
    uint8_t i;

    for (i = 0U; i < 8U; i++)
    {
        parity ^= (uint8_t)((value >> i) & 0x01U);
    }
#if YHM2712_ACMD_ACK_PARITY_INVERT
    parity ^= 0x01U;
#endif
    return (uint8_t)(parity & 0x01U);
}

#if YHM2712_ACMD_TX_VENDOR_NOP_ENABLE == 0U
YHM2712_ACMD_FORCE_INLINE uint32_t yhm2712_acmd_symbol_ticks(uint8_t symbol)
{
    if (symbol == 0U)
    {
        return s_yhm2712_acmd.logic0_ticks;
    }
    if (symbol == 1U)
    {
        return s_yhm2712_acmd.logic1_ticks;
    }
    return s_yhm2712_acmd.logicz_ticks;
}
#endif

#if YHM2712_ACMD_FACTORY_DETERMINISTIC_BYTE_TX == 1U
YHM2712_ACMD_FORCE_INLINE void yhm2712_acmd_send_data_symbol_factory_deterministic(
    uint8_t symbol)
{
    /*
     * Factory-only full-byte transmitter.  Complete the high-drive setup
     * before the falling edge, close each B0/B1 low pulse locally, then keep
     * output-high until the ACK receiver performs the sole release.
     */
    yhm2712_acmd_prepare_data_symbol_high_fast();
    yhm2712_acmd_delay_ticks(s_yhm2712_acmd.vendor_high_ticks);
    yhm2712_acmd_drive_data_low_width_closed(yhm2712_acmd_symbol_ticks(symbol));
    yhm2712_acmd_drive_high_fast();
}
#endif

#if YHM2712_ACMD_RX_VENDOR_SAMPLE_ENABLE == 0U
static uint8_t yhm2712_acmd_decode_symbol_ticks(uint32_t low_ticks)
{
    uint32_t threshold_0_1;
    uint32_t threshold_1_z;

    if (low_ticks < (s_yhm2712_acmd.logic0_ticks / 2U))
    {
        return YHM2712_ACMD_SYMBOL_UNKNOWN;
    }

    threshold_0_1 = (s_yhm2712_acmd.logic0_ticks + s_yhm2712_acmd.logic1_ticks) / 2U;
    threshold_1_z = (s_yhm2712_acmd.logic1_ticks + s_yhm2712_acmd.logicz_ticks) / 2U;

    if (low_ticks < threshold_0_1)
    {
        return 0U;
    }
    if (low_ticks < threshold_1_z)
    {
        return 1U;
    }
    return 2U;
}
#endif

YHM2712_ACMD_FORCE_INLINE void yhm2712_acmd_send_symbol_policy(
    uint8_t symbol,
    bool tail_interval,
    bool force_vendor_handoff_high,
    bool edge_timed)
{
#if YHM2712_ACMD_TX_VENDOR_NOP_ENABLE == 1U
    const uint32_t vendor_nop_low_loops =
        yhm2712_acmd_vendor_nop_symbol_loops(symbol);
#endif
#if YHM2712_ACMD_VENDOR_COMPAT_ENABLE
#if YHM2712_ACMD_TX_VENDOR_NOP_ENABLE == 1U
    yhm2712_acmd_prepare_data_symbol_high_fast();
#elif YHM2712_ACMD_TX_HIGH_REASSERT_COUNT > 1U
    if (symbol <= 1U)
    {
        yhm2712_acmd_prepare_data_symbol_high_fast();
    }
    else
    {
        yhm2712_acmd_drive_high_fast();
    }
#else
    yhm2712_acmd_drive_high_fast();
#endif
#if YHM2712_ACMD_TX_VENDOR_NOP_ENABLE == 1U
    /* Third-party path uses the B0 delay unit for every pre-symbol high. */
    yhm2712_acmd_vendor_nop_delay(YHM2712_ACMD_VENDOR_NOP_B0_LOOPS);
    yhm2712_acmd_drive_low_width_vendor_nop(vendor_nop_low_loops);
#else
    yhm2712_acmd_delay_ticks(s_yhm2712_acmd.vendor_high_ticks);
#if YHM2712_ACMD_TX_CLOSE_DATA_LOW_PULSE == 1U
    if (edge_timed && (symbol <= 1U))
    {
        yhm2712_acmd_drive_data_low_width_closed(
            yhm2712_acmd_symbol_ticks(symbol));
    }
    else
    {
        yhm2712_acmd_drive_low_width(yhm2712_acmd_symbol_ticks(symbol), edge_timed);
    }
#else
    yhm2712_acmd_drive_low_width(yhm2712_acmd_symbol_ticks(symbol), edge_timed);
#endif
#endif
    if (tail_interval || s_yhm2712_acmd.tail_drive_high ||
        force_vendor_handoff_high)
    {
        yhm2712_acmd_drive_high_fast();
    }
    else
    {
        yhm2712_acmd_release_fast();
    }
#else
    (void)force_vendor_handoff_high;
    yhm2712_acmd_drive_low_width(yhm2712_acmd_symbol_ticks(symbol), edge_timed);
    yhm2712_acmd_release_fast();
    if (tail_interval)
    {
        yhm2712_acmd_delay_ticks(s_yhm2712_acmd.interval_ticks);
    }
#endif
}

YHM2712_ACMD_FORCE_INLINE void yhm2712_acmd_send_symbol_ex(uint8_t symbol,
                                                            bool tail_interval)
{
    yhm2712_acmd_send_symbol_policy(symbol, tail_interval, false, false);
}

YHM2712_ACMD_FORCE_INLINE void yhm2712_acmd_send_symbol(uint8_t symbol)
{
    yhm2712_acmd_send_symbol_ex(symbol, true);
}

YHM2712_ACMD_FORCE_INLINE void yhm2712_acmd_vendor_reset_pulse_fast(bool edge_timed)
{
#if YHM2712_ACMD_VENDOR_COMPAT_ENABLE
#if YHM2712_ACMD_TX_VENDOR_NOP_ENABLE == 1U
    (void)edge_timed;
    yhm2712_acmd_prepare_data_symbol_high_fast();
    yhm2712_acmd_vendor_nop_delay(YHM2712_ACMD_VENDOR_NOP_B0_LOOPS);
    yhm2712_acmd_drive_low_width_vendor_nop(
        YHM2712_ACMD_VENDOR_NOP_BZ_LOOPS);
    yhm2712_acmd_vendor_nop_delay(YHM2712_ACMD_VENDOR_NOP_B0_LOOPS);
#else
    yhm2712_acmd_drive_high_fast();
    yhm2712_acmd_drive_low_width(s_yhm2712_acmd.logicz_ticks, edge_timed);
    yhm2712_acmd_drive_high_fast();
    yhm2712_acmd_delay_ticks(s_yhm2712_acmd.vendor_high_ticks);
#endif
#endif
}

#if !YHM2712_ACMD_VENDOR_COMPAT_ENABLE
YHM2712_ACMD_FORCE_INLINE yhm2712_acmd_status_t yhm2712_acmd_recv_symbol(
    yhm2712_acmd_symbol_sample_t *sample)
{
    uint32_t start;
    uint32_t low_start;
    uint32_t low_ticks;
    bool entry_high;

    if (sample == NULL)
    {
        return YHM2712_ACMD_STATUS_BAD_PARAM;
    }

    sample->low_cycles = 0U;
    sample->symbol = YHM2712_ACMD_SYMBOL_UNKNOWN;
    entry_high = yhm2712_acmd_line_is_high();
    sample->entry_line_level = entry_high ? 1U : 0U;
    sample->missed_fall = entry_high ? 0U : 1U;
    s_yhm2712_acmd.last_symbol = *sample;

    if (entry_high)
    {
        start = yhm2712_acmd_tick_now();
        while (yhm2712_acmd_line_is_high())
        {
            if (yhm2712_acmd_tick_delta(start, yhm2712_acmd_tick_now()) >
                s_yhm2712_acmd.edge_timeout_ticks)
            {
                return YHM2712_ACMD_STATUS_TIMEOUT_FALL;
            }
        }
    }
    else
    {
        low_start = yhm2712_acmd_tick_now();
        while (!yhm2712_acmd_line_is_high())
        {
            if (yhm2712_acmd_tick_delta(low_start, yhm2712_acmd_tick_now()) >
                s_yhm2712_acmd.edge_timeout_ticks)
            {
                sample->low_cycles =
                    yhm2712_acmd_tick_delta(low_start, yhm2712_acmd_tick_now());
                s_yhm2712_acmd.last_symbol = *sample;
                return YHM2712_ACMD_STATUS_RX_ENTRY_LOW;
            }
        }

        sample->low_cycles = yhm2712_acmd_tick_delta(low_start, yhm2712_acmd_tick_now());
        s_yhm2712_acmd.last_symbol = *sample;
        return YHM2712_ACMD_STATUS_RX_ENTRY_LOW;
    }

    low_start = yhm2712_acmd_tick_now();
    while (!yhm2712_acmd_line_is_high())
    {
        if (yhm2712_acmd_tick_delta(low_start, yhm2712_acmd_tick_now()) >
            s_yhm2712_acmd.edge_timeout_ticks)
        {
            sample->low_cycles =
                yhm2712_acmd_tick_delta(low_start, yhm2712_acmd_tick_now());
            s_yhm2712_acmd.last_symbol = *sample;
            return YHM2712_ACMD_STATUS_TIMEOUT_RISE;
        }
    }

    low_ticks = yhm2712_acmd_tick_delta(low_start, yhm2712_acmd_tick_now());
    sample->low_cycles = low_ticks;
    sample->symbol = yhm2712_acmd_decode_symbol_ticks(low_ticks);
    if (sample->symbol == YHM2712_ACMD_SYMBOL_UNKNOWN)
    {
        s_yhm2712_acmd.last_symbol = *sample;
        return YHM2712_ACMD_STATUS_BAD_SYMBOL;
    }

    s_yhm2712_acmd.last_symbol = *sample;
    return YHM2712_ACMD_STATUS_OK;
}
#endif

YHM2712_ACMD_FORCE_INLINE yhm2712_acmd_status_t yhm2712_acmd_recv_bit_sampled(
    yhm2712_acmd_symbol_sample_t *sample,
    uint32_t *fall_wait)
{
    uint32_t start;
    uint32_t low_start;
    uint32_t low_ticks = 0U;
#if YHM2712_ACMD_RX_VENDOR_SAMPLE_ENABLE == 0U
    const uint32_t read_threshold =
        yhm2712_acmd_ticks_from_tbit(YHM2712_ACMD_READ_WIDTH_THRESHOLD_TBIT);
#endif
    bool entry_high;

    if ((sample == NULL) || (fall_wait == NULL))
    {
        return YHM2712_ACMD_STATUS_BAD_PARAM;
    }

    sample->low_cycles = 0U;
    sample->symbol = YHM2712_ACMD_SYMBOL_UNKNOWN;
    *fall_wait = 0U;

    yhm2712_acmd_release_fast();
    entry_high = yhm2712_acmd_line_is_high();
    sample->entry_line_level = entry_high ? 1U : 0U;
    sample->missed_fall = entry_high ? 0U : 1U;

    if (entry_high)
    {
        start = yhm2712_acmd_tick_now();
        while (yhm2712_acmd_line_is_high())
        {
            *fall_wait = yhm2712_acmd_tick_delta(start, yhm2712_acmd_tick_now());
            if (*fall_wait > s_yhm2712_acmd.edge_timeout_ticks)
            {
                s_yhm2712_acmd.last_symbol = *sample;
                return YHM2712_ACMD_STATUS_TIMEOUT_FALL;
            }
        }
        *fall_wait = yhm2712_acmd_tick_delta(start, yhm2712_acmd_tick_now());
    }

#if YHM2712_ACMD_RX_VENDOR_SAMPLE_ENABLE == 1U
    /* Third-party read_bit(): sample once about 4.8 us after the falling edge. */
    yhm2712_acmd_vendor_nop_delay(YHM2712_ACMD_VENDOR_NOP_SA_LOOPS);
    sample->symbol = yhm2712_acmd_line_is_high() ? 0U : 1U;
    low_start = yhm2712_acmd_tick_now();
    while (!yhm2712_acmd_line_is_high())
    {
        low_ticks = yhm2712_acmd_tick_delta(low_start, yhm2712_acmd_tick_now());
        if (low_ticks > s_yhm2712_acmd.edge_timeout_ticks)
        {
            sample->low_cycles = low_ticks;
            s_yhm2712_acmd.last_symbol = *sample;
            return YHM2712_ACMD_STATUS_TIMEOUT_RISE;
        }
    }
    sample->low_cycles = low_ticks;
#else
    low_start = yhm2712_acmd_tick_now();
    while (!yhm2712_acmd_line_is_high())
    {
        low_ticks = yhm2712_acmd_tick_delta(low_start, yhm2712_acmd_tick_now());
        if (low_ticks > s_yhm2712_acmd.edge_timeout_ticks)
        {
            sample->low_cycles = low_ticks;
            s_yhm2712_acmd.last_symbol = *sample;
            return YHM2712_ACMD_STATUS_TIMEOUT_RISE;
        }
    }

    sample->low_cycles = yhm2712_acmd_tick_delta(low_start, yhm2712_acmd_tick_now());
    sample->symbol = (sample->low_cycles >= read_threshold) ? 1U : 0U;
#endif
    s_yhm2712_acmd.last_symbol = *sample;
    return YHM2712_ACMD_STATUS_OK;
}

#if YHM2712_ACMD_FACTORY_RAM_TIMING_ENABLE == 1U
DATA_RAM_FUNCTION
#endif
#if YHM2712_ACMD_ACK_SINGLE_RELEASE_HANDOFF == 1U
static yhm2712_acmd_status_t yhm2712_acmd_recv_ack_after_last_bit(
    uint8_t reg,
    yhm2712_acmd_stage_t ack_stage,
    uint8_t expected_ack)
#else
static yhm2712_acmd_status_t yhm2712_acmd_recv_ack_after_last_bit(
    uint8_t reg,
    yhm2712_acmd_stage_t ack_stage,
    uint8_t value)
#endif
{
    uint32_t start;
    uint32_t low_start;
    uint32_t high_wait = 0U;
    uint32_t fall_wait = 0U;
    uint32_t low_ticks = 0U;
    uint32_t turnaround_tail_ticks = 0U;
#if YHM2712_ACMD_FACTORY_BOUNDARY_DIAG_ENABLE == 1U
    uint32_t release_start_ticks = 0U;
    uint32_t last_high_to_release_ticks = 0U;
    uint32_t release_write_ticks = 0U;
    uint32_t release_to_low_ticks = 0U;
    uint8_t release_line_level = 0U;
    bool boundary_valid = false;
#endif
    uint8_t ack = YHM2712_ACMD_SYMBOL_UNKNOWN;
#if YHM2712_ACMD_ACK_SINGLE_RELEASE_HANDOFF == 0U
    uint8_t expected_ack = YHM2712_ACMD_SYMBOL_UNKNOWN;
#endif
    yhm2712_acmd_status_t status = YHM2712_ACMD_STATUS_OK;
    yhm2712_acmd_ack_path_t path = YHM2712_ACMD_ACK_PATH_NORMAL;
    yhm2712_acmd_symbol_sample_t sample;
#if YHM2712_ACMD_FACTORY_ACK_POLL_DIAG_ENABLE == 1U
    yhm2712_acmd_ack_poll_diag_t poll_diag;
    uint32_t low_poll_count = 0U;
    uint32_t max_tick_step = 0U;
    uint32_t diag_low_start_tick = 0U;
    uint32_t diag_rise_tick = 0U;
    uint32_t poll_tick;
    uint32_t previous_tick;
    uint32_t tick_step;
#endif

#if YHM2712_ACMD_ACK_SINGLE_RELEASE_HANDOFF == 1U
    /*
     * Factory handoff contract: expected_ack was computed before any data bit
     * was sent. The final master bit remains actively high, so the first
     * physical operation after that bit is the one output-to-input release.
     * Keep this window free of parity work, trace timestamps and GPIO reads.
     */
#if YHM2712_ACMD_FACTORY_BOUNDARY_DIAG_ENABLE == 1U
    release_start_ticks = yhm2712_acmd_tick_now();
#endif
    yhm2712_acmd_release_fast();
    start = yhm2712_acmd_tick_now();
#if YHM2712_ACMD_FACTORY_BOUNDARY_DIAG_ENABLE == 1U
    if (s_yhm2712_acmd.last_symbol_high_valid)
    {
        last_high_to_release_ticks = yhm2712_acmd_tick_delta(
            s_yhm2712_acmd.last_symbol_high_tick, release_start_ticks);
        release_write_ticks = yhm2712_acmd_tick_delta(release_start_ticks, start);
        boundary_valid = true;
    }
#endif
    if (!yhm2712_acmd_line_is_high())
    {
        /*
         * The line is already observed low after the unique release.  Do not
         * use the pre-observation release timestamp as the low-width origin:
         * it can include an unknown amount of high-line time before the
         * slave ACK falling edge.  Start from the first post-observation tick
         * instead, just as the normal falling-edge path below does.
         */
        low_start = yhm2712_acmd_tick_now();
#if YHM2712_ACMD_FACTORY_BOUNDARY_DIAG_ENABLE == 1U
        release_line_level = 0U;
#endif
        sample.entry_line_level = 0U;
        sample.missed_fall = 1U;
    }
    else
    {
        sample.entry_line_level = 1U;
#if YHM2712_ACMD_FACTORY_BOUNDARY_DIAG_ENABLE == 1U
        release_line_level = 1U;
#endif
        sample.missed_fall = 0U;
        /*
         * Detect the falling edge first, then take the low-width origin in
         * the same polling iteration.  Starting at a tick read that precedes
         * the DATAIN observation can include time while the line was still
         * high, stretching a short ACK0 across the 0/1 threshold.
         */
        while (yhm2712_acmd_line_is_high())
        {
            fall_wait = yhm2712_acmd_tick_delta(start, yhm2712_acmd_tick_now());
            if (fall_wait > s_yhm2712_acmd.edge_timeout_ticks)
            {
                status = YHM2712_ACMD_STATUS_TIMEOUT_FALL;
                goto ack_done;
            }
        }
        low_start = yhm2712_acmd_tick_now();
        fall_wait = yhm2712_acmd_tick_delta(start, low_start);
    }
#if YHM2712_ACMD_FACTORY_BOUNDARY_DIAG_ENABLE == 1U
    release_to_low_ticks = yhm2712_acmd_tick_delta(start, low_start);
#endif
#if YHM2712_ACMD_ACK_PREAMBLE_FILTER_ENABLE == 1U
ack_wait_next_fall:
#endif
#else
    yhm2712_acmd_release_fast();
    expected_ack = yhm2712_acmd_parity_bit(value);
    if ((GPIO->DATADIR & s_yhm2712_acmd.gpio_pin) != 0U)
    {
        status = YHM2712_ACMD_STATUS_RELEASE_TIMEOUT;
        goto ack_done;
    }

    if (!yhm2712_acmd_line_is_high())
    {
        /*
         * The last master address bit can leave a short low tail after the
         * GPIO is released. It has no observable falling edge, so it cannot
         * be decoded as the slave ACK. Drain it, then capture the next full
         * ACK pulse as an independent protocol symbol.
         */
        path = YHM2712_ACMD_ACK_PATH_TURNAROUND_TAIL;
        low_start = yhm2712_acmd_tick_now();
        while (!yhm2712_acmd_line_is_high())
        {
            turnaround_tail_ticks =
                yhm2712_acmd_tick_delta(low_start, yhm2712_acmd_tick_now());
            if (turnaround_tail_ticks > s_yhm2712_acmd.edge_timeout_ticks)
            {
                path = YHM2712_ACMD_ACK_PATH_STUCK_LOW;
                status = YHM2712_ACMD_STATUS_RELEASE_TIMEOUT;
                goto ack_done;
            }
        }
        turnaround_tail_ticks =
            yhm2712_acmd_tick_delta(low_start, yhm2712_acmd_tick_now());
        s_yhm2712_acmd.ack_turnaround_tail_count++;
    }
    yhm2712_acmd_trace_record_release(ack_stage, turnaround_tail_ticks);

    start = yhm2712_acmd_tick_now();
    while (yhm2712_acmd_line_is_high())
    {
        fall_wait = yhm2712_acmd_tick_delta(start, yhm2712_acmd_tick_now());
        if (fall_wait > s_yhm2712_acmd.edge_timeout_ticks)
        {
            status = YHM2712_ACMD_STATUS_TIMEOUT_FALL;
            goto ack_done;
        }
    }
    fall_wait = yhm2712_acmd_tick_delta(start, yhm2712_acmd_tick_now());
    low_start = yhm2712_acmd_tick_now();
#endif
#if YHM2712_ACMD_RX_VENDOR_SAMPLE_ENABLE == 1U
    /* Match third-party wait_ack/read_bit classification at the 4.8 us point. */
    yhm2712_acmd_vendor_nop_delay(YHM2712_ACMD_VENDOR_NOP_SA_LOOPS);
    ack = yhm2712_acmd_line_is_high() ? 0U : 1U;
    low_start = yhm2712_acmd_tick_now();
    while (!yhm2712_acmd_line_is_high())
    {
        low_ticks = yhm2712_acmd_tick_delta(low_start, yhm2712_acmd_tick_now());
        if (low_ticks > s_yhm2712_acmd.edge_timeout_ticks)
        {
            path = YHM2712_ACMD_ACK_PATH_STUCK_LOW;
            status = YHM2712_ACMD_STATUS_RELEASE_TIMEOUT;
            goto ack_done;
        }
    }
#else
#if YHM2712_ACMD_FACTORY_ACK_POLL_DIAG_ENABLE == 1U
    diag_low_start_tick = low_start;
    previous_tick = low_start;
    while (!yhm2712_acmd_line_is_high())
    {
        poll_tick = yhm2712_acmd_tick_now();
        low_ticks = yhm2712_acmd_tick_delta(low_start, poll_tick);
        tick_step = yhm2712_acmd_tick_delta(previous_tick, poll_tick);
        if (tick_step > max_tick_step)
        {
            max_tick_step = tick_step;
        }
        previous_tick = poll_tick;
        low_poll_count++;
        if (low_ticks > s_yhm2712_acmd.edge_timeout_ticks)
        {
            path = YHM2712_ACMD_ACK_PATH_STUCK_LOW;
            status = YHM2712_ACMD_STATUS_RELEASE_TIMEOUT;
            goto ack_done;
        }
    }
    diag_rise_tick = yhm2712_acmd_tick_now();
    tick_step = yhm2712_acmd_tick_delta(previous_tick, diag_rise_tick);
    if (tick_step > max_tick_step)
    {
        max_tick_step = tick_step;
    }
    low_ticks = yhm2712_acmd_tick_delta(low_start, diag_rise_tick);
#else
    while (!yhm2712_acmd_line_is_high())
    {
        low_ticks = yhm2712_acmd_tick_delta(low_start, yhm2712_acmd_tick_now());
        if (low_ticks > s_yhm2712_acmd.edge_timeout_ticks)
        {
            path = YHM2712_ACMD_ACK_PATH_STUCK_LOW;
            status = YHM2712_ACMD_STATUS_RELEASE_TIMEOUT;
            goto ack_done;
        }
    }
    low_ticks = yhm2712_acmd_tick_delta(low_start, yhm2712_acmd_tick_now());
#endif
#if YHM2712_ACMD_ACK_PREAMBLE_FILTER_ENABLE == 1U
    /*
     * With VIN present STACMD is also the charger's status output and is
     * internally pulled low while charging.  At the push-pull-to-input
     * handoff this can expose a very short status/preamble low before the
     * slave emits its complete ACK symbol.  It is shorter than the existing
     * minimum decodable B0 width, so skip it without changing any protocol
     * threshold and keep the original release-to-edge timeout as the bound.
     */
    if (s_yhm2712_acmd.tail_drive_high &&
        (low_ticks < (s_yhm2712_acmd.logic0_ticks / 2U)))
    {
        s_yhm2712_acmd.ack_preamble_filter_count++;
#if YHM2712_ACMD_FACTORY_ACK_POLL_DIAG_ENABLE == 1U
        low_poll_count = 0U;
        max_tick_step = 0U;
        diag_low_start_tick = 0U;
        diag_rise_tick = 0U;
#endif
        while (yhm2712_acmd_line_is_high())
        {
            fall_wait = yhm2712_acmd_tick_delta(start, yhm2712_acmd_tick_now());
            if (fall_wait > s_yhm2712_acmd.edge_timeout_ticks)
            {
                status = YHM2712_ACMD_STATUS_TIMEOUT_FALL;
                goto ack_done;
            }
        }
        low_start = yhm2712_acmd_tick_now();
        fall_wait = yhm2712_acmd_tick_delta(start, low_start);
        sample.entry_line_level = 1U;
        sample.missed_fall = 0U;
        goto ack_wait_next_fall;
    }
#endif
    ack = yhm2712_acmd_decode_symbol_ticks(low_ticks);
    if (ack > 1U)
    {
        status = YHM2712_ACMD_STATUS_BAD_SYMBOL;
        goto ack_done;
    }
#endif

    if (ack != expected_ack)
    {
        status = YHM2712_ACMD_STATUS_ACK_MISMATCH;
    }

#if YHM2712_ACMD_FACTORY_RAM_TIMING_ENABLE == 1U
    if ((status == YHM2712_ACMD_STATUS_OK) &&
        (ack_stage == YHM2712_ACMD_STAGE_ADDR_R_ACK))
    {
        /*
         * A successful read-address ACK is immediately followed by the first
         * returned data bit.  Leave the RAM ACK hot path before recording the
         * optional ACK/release diagnostics; those Flash-side stores previously
         * consumed the first data-bit window on the first cold transaction.
         * recv_byte_send_ack() records the returned symbol (or its timeout), so
         * no failure evidence is lost at the data boundary.
         */
        return YHM2712_ACMD_STATUS_OK;
    }
#endif

ack_done:
#if YHM2712_ACMD_FACTORY_ACK_POLL_DIAG_ENABLE == 1U
    poll_diag.valid = true;
    poll_diag.low_poll_count = low_poll_count;
    poll_diag.max_tick_step = max_tick_step;
    poll_diag.low_start_tick = diag_low_start_tick;
    poll_diag.rise_tick = diag_rise_tick;
    poll_diag.primask = (__get_PRIMASK() != 0U) ? 1U : 0U;
    poll_diag.dir_m = ((GPIO->DATADIR & s_yhm2712_acmd.gpio_pin) != 0U) ? 1U : 0U;
    poll_diag.datasrc_m = ((GPIO->DATASRC & s_yhm2712_acmd.gpio_pin) != 0U) ? 1U : 0U;
    poll_diag.debounce_m = ((GPIO->DEBOUNCE & s_yhm2712_acmd.gpio_pin) != 0U) ? 1U : 0U;
    poll_diag.lssync_m = ((GPIO->LSSYNC & s_yhm2712_acmd.gpio_pin) != 0U) ? 1U : 0U;
#endif
    sample.low_cycles = low_ticks;
    sample.symbol = ack;
#if YHM2712_ACMD_ACK_SINGLE_RELEASE_HANDOFF == 0U
    sample.entry_line_level =
        (turnaround_tail_ticks != 0U) ? 0U : 1U;
    sample.missed_fall =
        (turnaround_tail_ticks != 0U) ? 1U : 0U;
#endif
    s_yhm2712_acmd.last_symbol = sample;
    yhm2712_acmd_trace_record_ack(ack_stage, path, status, high_wait, fall_wait,
                                  low_ticks, turnaround_tail_ticks, expected_ack, ack
#if YHM2712_ACMD_FACTORY_ACK_POLL_DIAG_ENABLE == 1U
                                  , &poll_diag
#endif
                                 );
#if YHM2712_ACMD_FACTORY_BOUNDARY_DIAG_ENABLE == 1U
    if (boundary_valid && (status != YHM2712_ACMD_STATUS_OK))
    {
        DBG_DIRECT("[YHM2712][BOUNDARY] r=%02x st=%u lh=%lu rw=%lu rf=%lu re=%u",
                   (uint32_t)reg,
                   (uint32_t)ack_stage,
                   (unsigned long)last_high_to_release_ticks,
                   (unsigned long)release_write_ticks,
                   (unsigned long)release_to_low_ticks,
                   (uint32_t)release_line_level);
    }
#endif
    if (!s_yhm2712_acmd.release_trace.valid)
    {
        yhm2712_acmd_trace_record_release(ack_stage, turnaround_tail_ticks);
    }
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        yhm2712_acmd_trace_record_fail(reg, ack_stage, status);
    }
    return status;
}

#if YHM2712_ACMD_FACTORY_RAM_TIMING_ENABLE == 1U
DATA_RAM_FUNCTION
static yhm2712_acmd_status_t
#else
YHM2712_ACMD_FORCE_INLINE yhm2712_acmd_status_t
#endif
yhm2712_acmd_send_byte_check_ack_policy(
    uint8_t reg,
    yhm2712_acmd_stage_t ack_stage,
    uint8_t value,
    bool force_vendor_handoff_high,
    bool edge_timed)
{
    uint8_t bit;
    yhm2712_acmd_status_t status;

#if YHM2712_ACMD_ACK_SINGLE_RELEASE_HANDOFF == 1U
    /* Precompute while the bus is idle; never do parity work at handoff. */
    const uint8_t expected_ack = yhm2712_acmd_parity_bit(value);
#endif

#if YHM2712_ACMD_FACTORY_DETERMINISTIC_BYTE_TX == 1U
    (void)force_vendor_handoff_high;
    (void)edge_timed;
    for (bit = 0U; bit < YHM2712_ACMD_DATA_BITS; bit++)
    {
        yhm2712_acmd_send_data_symbol_factory_deterministic(
            (uint8_t)((value >> (7U - bit)) & 0x01U));
    }
#else
#if YHM2712_ACMD_ACK_SINGLE_RELEASE_HANDOFF == 1U
    /* Every Factory address/data byte keeps its final bit actively high. */
    force_vendor_handoff_high = true;
#endif
    for (bit = 0U; bit < 7U; bit++)
    {
        yhm2712_acmd_send_symbol_policy(
            (uint8_t)((value >> (7U - bit)) & 0x01U), true, false,
            edge_timed);
    }
    yhm2712_acmd_send_symbol_policy((uint8_t)(value & 0x01U), false,
                                    force_vendor_handoff_high,
                                    edge_timed);
#endif

#if YHM2712_ACMD_FACTORY_BOUNDARY_DIAG_ENABLE == 1U
    s_yhm2712_acmd.last_symbol_high_tick = yhm2712_acmd_tick_now();
    s_yhm2712_acmd.last_symbol_high_valid = true;
#endif
#if YHM2712_ACMD_ACK_SINGLE_RELEASE_HANDOFF == 1U
    status = yhm2712_acmd_recv_ack_after_last_bit(reg, ack_stage, expected_ack);
#else
    status = yhm2712_acmd_recv_ack_after_last_bit(reg, ack_stage, value);
#endif
    if (status != YHM2712_ACMD_STATUS_OK)
    {
        return status;
    }

#if YHM2712_ACMD_LAST_BIT_TAIL_INTERVAL
    yhm2712_acmd_delay_ticks(s_yhm2712_acmd.interval_ticks);
#endif
    return YHM2712_ACMD_STATUS_OK;
}

YHM2712_ACMD_FORCE_INLINE yhm2712_acmd_status_t yhm2712_acmd_send_byte_check_ack(
    uint8_t reg,
    yhm2712_acmd_stage_t ack_stage,
    uint8_t value)
{
    return yhm2712_acmd_send_byte_check_ack_policy(reg, ack_stage, value,
                                                    false, false);
}

#if YHM2712_ACMD_FACTORY_RAM_TIMING_ENABLE == 1U
YHM2712_ACMD_FORCE_INLINE void yhm2712_acmd_data_trace_prepare(void)
{
    uint8_t clear_idx;

    s_yhm2712_acmd.data_trace.valid = true;
    s_yhm2712_acmd.data_trace.status = YHM2712_ACMD_STATUS_OK;
    s_yhm2712_acmd.data_trace.value = 0U;
    s_yhm2712_acmd.data_trace.bit_count = 0U;
    for (clear_idx = 0U; clear_idx < YHM2712_ACMD_DATA_BITS; clear_idx++)
    {
        s_yhm2712_acmd.data_trace.symbols[clear_idx] = YHM2712_ACMD_SYMBOL_UNKNOWN;
        s_yhm2712_acmd.data_trace.lows[clear_idx] = 0U;
        s_yhm2712_acmd.data_trace.fall_waits[clear_idx] = 0U;
    }
}
#endif

#if YHM2712_ACMD_FACTORY_RAM_TIMING_ENABLE == 1U
DATA_RAM_FUNCTION
static yhm2712_acmd_status_t
#else
YHM2712_ACMD_FORCE_INLINE yhm2712_acmd_status_t
#endif
yhm2712_acmd_recv_byte_send_ack(uint8_t *value, bool nack)
{
    uint8_t bit;
#if YHM2712_ACMD_FACTORY_RAM_TIMING_ENABLE == 0U
    uint8_t clear_idx;
#endif
    uint8_t symbol;
    uint8_t data = 0U;
    uint8_t ack;
    yhm2712_acmd_symbol_sample_t sample;
    yhm2712_acmd_status_t status;
    uint32_t fall_wait = 0U;

    if (value == NULL)
    {
        return YHM2712_ACMD_STATUS_BAD_PARAM;
    }

#if YHM2712_ACMD_FACTORY_RAM_TIMING_ENABLE == 0U
    s_yhm2712_acmd.data_trace.valid = true;
    s_yhm2712_acmd.data_trace.status = YHM2712_ACMD_STATUS_OK;
    s_yhm2712_acmd.data_trace.value = 0U;
    s_yhm2712_acmd.data_trace.bit_count = 0U;
    for (clear_idx = 0U; clear_idx < YHM2712_ACMD_DATA_BITS; clear_idx++)
    {
        s_yhm2712_acmd.data_trace.symbols[clear_idx] = YHM2712_ACMD_SYMBOL_UNKNOWN;
        s_yhm2712_acmd.data_trace.lows[clear_idx] = 0U;
        s_yhm2712_acmd.data_trace.fall_waits[clear_idx] = 0U;
    }
#endif

    for (bit = 0U; bit < 8U; bit++)
    {
#if YHM2712_ACMD_VENDOR_COMPAT_ENABLE
        status = yhm2712_acmd_recv_bit_sampled(&sample, &fall_wait);
#else
        status = yhm2712_acmd_recv_symbol(&sample);
        fall_wait = 0U;
#endif
        if (status != YHM2712_ACMD_STATUS_OK)
        {
            s_yhm2712_acmd.data_trace.status = status;
            s_yhm2712_acmd.data_trace.value = data;
            s_yhm2712_acmd.data_trace.bit_count = bit;
            return status;
        }
        symbol = sample.symbol;
        s_yhm2712_acmd.data_trace.symbols[bit] = symbol;
        s_yhm2712_acmd.data_trace.lows[bit] = sample.low_cycles;
        s_yhm2712_acmd.data_trace.fall_waits[bit] = fall_wait;
        s_yhm2712_acmd.data_trace.bit_count = (uint8_t)(bit + 1U);
        if (symbol > 1U)
        {
            s_yhm2712_acmd.data_trace.status = YHM2712_ACMD_STATUS_BAD_SYMBOL;
            s_yhm2712_acmd.data_trace.value = data;
            return YHM2712_ACMD_STATUS_BAD_SYMBOL;
        }
        data = (uint8_t)((data << 1U) | symbol);
        s_yhm2712_acmd.data_trace.value = data;
    }

    ack = yhm2712_acmd_parity_bit(data);
    if (nack)
    {
        ack ^= 0x01U;
    }
#if YHM2712_ACMD_VENDOR_COMPAT_ENABLE
    if (!nack)
    {
        yhm2712_acmd_delay_ticks(s_yhm2712_acmd.interval_ticks);
        yhm2712_acmd_send_symbol_ex(ack, true);
    }
#else
    yhm2712_acmd_delay_ticks(s_yhm2712_acmd.interval_ticks);
    yhm2712_acmd_send_symbol_ex(ack, true);
#endif
    *value = data;
    s_yhm2712_acmd.data_trace.status = YHM2712_ACMD_STATUS_OK;
    s_yhm2712_acmd.data_trace.value = data;
    return YHM2712_ACMD_STATUS_OK;
}

