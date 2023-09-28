<img src="./images/IFX_LOGO_600.gif" align="right" width="150"/>

# Ethernet Connection Manager (ECM) for KIT_T2G-B-H_LITE
**This library has same functionality with original [ECM](https://github.com/Infineon/ethernet-connection-manager) except for the implementation regarding the Ethernet MAC and PHY handling which are made to fit with use on [KIT_T2G-B-H_LITE](https://www.infineon.com/cms/en/product/evaluation-boards/kit_t2g-b-h_lite/).**

## Base Library
- [Ethernet Connection Manager](https://github.com/Infineon/ethernet-connection-manager) : release-v1.0.0 (SHA-1: d99428c8d2b8d5512e8ba2dcb2e76d59825abb13)

## Differences

**MAC layer**
<table border="1" style="border-collapse: collapse">
<thead><tr>
<th></th><th>Base Library</th><th>This Library</th></tr></thead>
<tbody>
<tr><td><code>MAC channel</code></td><td>CH1</td><td>CH0</td></tr>
<tr><td><code>PHY Interface</code></td><td>RGMII</td><td>RMII</td></tr>
</tbody>
</table>
<br>

**PHY layer**
<table border="1" style="border-collapse: collapse">
<thead><tr>
<th></th><th>Base Library</th><th>This Library</th></tr></thead>
<tbody>
<tr><td><code>PHY chip</code></td><td>DP83867IR</td><td>DP83825I</td></tr>
</tbody>
</table>

## References  

ModusToolbox™ is available online:
- <https://www.infineon.com/modustoolbox>

Associated TRAVEO™ T2G MCUs can be found on:
- <https://www.infineon.com/cms/en/product/microcontroller/32-bit-traveo-t2g-arm-cortex-microcontroller/>

More code examples can be found on the GIT repository:
- [TRAVEO™ T2G Code examples](https://github.com/orgs/Infineon/repositories?q=mtb-t2g-&type=all&language=&sort=)

For additional trainings, visit our webpage:  
- [TRAVEO™ T2G trainings](https://www.infineon.com/cms/en/product/microcontroller/32-bit-traveo-t2g-arm-cortex-microcontroller/32-bit-traveo-t2g-arm-cortex-for-body/traveo-t2g-cyt4bf-series/#!trainings)

For questions and support, use the TRAVEO™ T2G Forum:  
- <https://community.infineon.com/t5/TRAVEO-T2G/bd-p/TraveoII>  
