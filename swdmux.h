#ifndef SWDMUX_H
#define SWDMUX_H

// SWD mux position select.
//
// Selects which physical connector's SWD lines (SWCLK/SWDIO) are routed to
// the shared SWD bus this wrapper bitbangs on, via an I2C GPIO expander.
// This mirrors an existing Go driver for the same mux hardware: same I2C
// device/address/registers, same logical position numbering (0 = extender,
// 1..12 = node, through the same lookup table), same retry count.

namespace swdmux {

// Which I2C bus and expander address the mux lives on, and whether there is a
// mux at all. Called once at startup, before any of the calls below;
// i2c_dev must outlive the process's use of the mux.
//
// With enabled false (boards that wire SWD straight through to a single
// target) the startup select is skipped, DAP_Vendor0 is rejected, and DAP_Info
// stops reporting a position.
void init(const char *i2c_dev, unsigned int i2c_addr, bool enabled);
bool is_enabled(void);

// Select a mux position (0 = extender, 1..12 = node), retrying on I2C errors.
// Returns false if the mux is disabled, the position is out of range, or every
// attempt failed.
bool select_pos(unsigned int pos);

// Last position selected successfully, or -1 if none has been selected yet.
int current_pos(void);

}  // namespace swdmux

#endif  // SWDMUX_H
