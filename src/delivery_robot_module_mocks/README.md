# delivery_robot_module_mocks

Drop-in mock Action Servers for `/arm/pick_tray`, `/cargo/store_tray`,
`/cargo/retrieve_tray`, and `/cargo/eject_tray`. The mocks publish the same
module and cargo state topics as the future physical modules. While a mock
mechanism action is active, `transport_safe=false`, allowing the mission and
chassis safety gates to be tested before hardware integration.
