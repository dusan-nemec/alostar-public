# APVV SK-IL-RD-0002: Advanced Localization Sensors and Techniques for Autonomous vehicles and Robots

## Anotácia
Integrácia globálnej satelitnej navigácie, odometrov a inerciálnych snímačov je štandardne používanou technikou na lokalizáciu pozemných vozidiel a mobilných robotov. Viacero štúdií naznačuje, že inerciálne snímače umiestnené na kolesách vozidla môžu zvýšiť presnosť navigačného systému. Popritom techniky hlbokého učenia boli aplikované vo viacerých navigačných úlohách, pričom vykazovali lepšie vlastnosti v porovnaní s metódami založenými na matematických modeloch. V rámci tohto projektu navrhujeme vyvinúť softvérový rámec (framework) využívajúci strojové učenie, ktorý by umožnil lepšie využiť potenciál inerciálnych snímačov umiestnených na kolesách vozidla. Budú preskúmané možnosti optimálneho umiestnenia inerciálnych snímačov a následne bude odvodený a implementovaný hybridný navigačný filter využívajúci strojové učenie. Navrhnuté algoritmy budú validované pomocou experimentov v reálnych podmienkach s využitím pozemných vozidiel a mobilných kolesových robotov.

## Štruktúra repozitáru
### models
Priečinok obsahuje softvérovú implementáciu simulačných modelov kolesových robotov. Je rozdelený na podpriečinky:
- `kinematics`- obsahuje kinematický model diferenciálne riadeného robota v jazyku MATLAB
- `dynamics` - obsahuje model dynamiky 4-kolesového šmykom riadeného robota v jazyku MATLAB
- `wheel` - obsahuje simulačný model IMU snímača namontovaného na kolese robota v jazyku Python

### pcb
Priečinok obsahuje podklady na výrobu DPS prototypu meracej jednotky. Obsahuje súbory:
- `wheel.sch` - elektrická schéma zapojenia pre EAGLE
- `wheel.brd` - rozmiestnenie komponentov pre EAGLE
- `wheel-bottom.pdf` - ukážka spodnej vrstvy DPS vo formáte PDF
- `wheel-top.pdf` - ukážka vrchnej vrstvy DPS vo formáte PDF
- `wheel-scheme.pdf` - elektrická schéma zapojenia vo formáte PDF

### firmware
Priečinok obsahuje zdrojový kód riadiaceho programu meracej jednotky navrhnutej v rámci projektu. Jednotka je vybavená riadiacim mikropočítačom typu ESP32 a je programovateľná v prostredí Arduino IDE. Obsahuje podpriečinky:
- `lib` - priečinok obsahuje custom knižnice Arduino. Pre úspešné skompilovanie firmvéru je potrebné tieto knižnice skopírovať do priečinka `~\Documents\Arduino\libraries`
- `wheel_sensor` - priečinok obsahuje projekt firmvéru v jazyku C++, otvoriteľný v prostredí Arduino IDE

### protocol
Priečinok obsahuje textový opis komunikačného protokolu meracej jednotky vo formáte PDF.

### logger
Priečinok obsahuje zdrojový kód jednoduchého softvéru na ukladanie údajov z meracích jednotiek napísaný v jazyku Python.
- `logger.py` - hlavný spustiteľný skript
- `tcp.py` - klient pre TCP/IP komunikáciu
- `udp.py` - klient pre UDP/IP komunikáciu