# CubeMX regeneration rules

- This repository is a CubeMX-generated STM32 project. Keep application logic in
  separate application-owned files (motor, control_udp, ether_phy, ether_msp,
  board_identity, app_time, app_scheduler and their headers).
- Edit generated C/H files only inside an existing matching USER CODE region.
  Never patch the generated ethernet_link_check_state() body: CubeMX erases it.
  PHY polling belongs to app_scheduler -> EthernetPhy_Process -> EthernetPhy_Check.
- Make pin labels, pulls, peripheral clocks and hardware timer configuration in
  ABS_CONTROLLER_FW.ioc; generated GPIO/peripheral code is not authoritative.
- Retain main.c USER CODE hooks for identity initialization, motor/UDP startup,
  TIM6 start/tick and scheduler processing. Keep the IP/MAC override hooks in
  lwip.c/ethernetif.c USER CODE sections. Keep the generated lwIP periodic PHY
  handler disabled through its existing USER CODE guards to avoid dual polling.
- After regeneration, review git diff, verify these hooks, then build. Check
  for duplicate HAL_ETH_MspInit if CubeMX starts generating its own MSP body.
- Test relevant application behavior. Scheduler tests must target the application
  PHY entry point, independent of the generated BSP link-check function.
- Keep CubeMX's "Keep User Code when re-generating" enabled. Do not regenerate
  or flash firmware merely as part of a source review without user direction.
