# AUTO / TEACH – gałąź `dev`

## Założenie HOME

Robot nie wykonuje automatycznego homingu. Przed włączeniem zasilania lub resetem należy ręcznie ustawić mechanikę w pozycji odpowiadającej:

- Arm 1: `HOME_ARM1_DEG`
- Arm 2: `HOME_ARM2_DEG`
- Z: `HOME_Z_MM`

Po starcie ESP32 pozycje liczników krokowych są ustawiane na te wartości.

## Tryby pracy

Sterownik ma dwa niezależne poziomy trybów:

- `OperatingMode::MANUAL` / `OperatingMode::AUTO`
- w MANUAL: `ControlMode::JOINT` / `ControlMode::TOOL`

Dzięki temu JOINT/TOOL opisuje sposób ręcznego prowadzenia robota, a AUTO jest osobnym trybem pracy.

## Przyciski

### Przycisk MODE na PCB

Przełącza `MANUAL <-> AUTO`.

Zmiana jest odrzucana, gdy osie są w ruchu.

### Przycisk TEACH na PCB

Działa tylko w MANUAL i przy zatrzymanych osiach.

- krótkie naciśnięcie: zapis aktualnego punktu,
- przytrzymanie >= 1.8 s: wyczyszczenie całego programu.

Punkt zapisuje:

- rzeczywistą pozycję Arm 1,
- rzeczywistą pozycję Arm 2,
- rzeczywistą pozycję Z,
- pozycję serwa obrotu chwytaka,
- pozycję serwa chwytaka,
- czas postoju `dwellMs`.

Program może zawierać maksymalnie 20 punktów.

Punkty są zapisywane w pamięci NVS ESP32 i pozostają po wyłączeniu zasilania.

## Pad

### MANUAL

- `START` – JOINT / TOOL,
- lewy analog – Arm1/Arm2 albo X/Y TCP,
- prawy Y – Z,
- prawy X – obrót chwytaka,
- A/B – otwieranie/zamykanie chwytaka.

Po połączeniu pada wymagane jest 300 ms neutralnych wejść przed uzbrojeniem ruchu.

### AUTO

`START` działa jako:

- IDLE -> START,
- RUN -> PAUSE,
- PAUSED -> RESUME.

Regulacja prędkości całego ruchu w AUTO:

- D-pad góra: `+10%`,
- D-pad dół: `-10%`,
- D-pad lewo: powrót do `100%`,
- zakres regulacji: `50% ... 250%`,
- regulacja skaluje Arm 1, Arm 2 oraz oś Z,
- nowa wartość zaczyna obowiązywać od następnego punktu PTP; bieżący ruch nie jest przeliczany w locie.

Bazowo oba ramiona w AUTO mają `20 deg/s`, a oś Z `1.5 mm/s`. Przykładowo przy `200%` ramiona pracują z prędkością bazową `40 deg/s`, a Z z prędkością bazową `3.0 mm/s`. Sterownik nie nakłada dodatkowego programowego limitu częstotliwości STEP; rzeczywista granica wynika z możliwości napędu, sterownika i mechaniki.

Utrata pada podczas AUTO przełącza program w PAUSE.

## Cykl AUTO

AUTO odtwarza zapisane punkty jako ruch PTP. Dla każdego punktu:

1. ustawiane są cele serw,
2. obliczane są odległości wszystkich trzech osi,
3. dobierane są częstotliwości STEP tak, aby osie w przybliżeniu kończyły ruch w tym samym czasie,
4. sterownik czeka nieblokująco na zakończenie wszystkich osi,
5. wykonywany jest `dwellMs`,
6. uruchamiany jest kolejny punkt.

Domyślnie program pracuje w pętli (`AUTO_LOOP_PROGRAM = true`).

## Bezpieczeństwo

- E-STOP natychmiast zatrzymuje osie i ustawia AUTO w `FAULT`.
- Po E-STOP wymagany jest restart sterownika.
- AUTO nie jest zależne od `gamepadArmed` – blokada `gamepadArmed` dotyczy wyłącznie ruchu ręcznego.
- Do AUTO wykorzystywana jest rzeczywista pozycja liczona z `FastAccelStepper::getCurrentPosition()`.
- Oprogramowanie nie nakłada dodatkowego twardego limitu częstotliwości STEP; prędkość wynika z nastaw ruchu i procentu AUTO.

## Kolejny etap

Obecna wersja AUTO realizuje PTP. Następnym rozszerzeniem może być interpolacja `LIN`, w której TCP jest dzielony na krótkie odcinki XYZ, a dla kolejnych próbek wykonywana jest kinematyka odwrotna.
