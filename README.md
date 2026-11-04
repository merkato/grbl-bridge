# GRBL Bridge

Wysokowydajny mostek WebSocket / HTTP do sterowników GRBL 1.1 uruchamiany na Raspberry Pi.

## Funkcje
- **Przeliczanie współrzędnych**: Automatyczne wyliczanie `WPos` z raportów `MPos` i wektora `WCO`.
- **Wielowątkowy bufor strumieniowy**: Obsługa komend czasu rzeczywistego (`!`, `~`, `\x18`, `\x85`) bez opóźnień kolejki bufora planisty. Jedna linia „w locie”, więc odpowiedzi `ok` nie mieszają się między zadaniem a komendami ręcznymi.
- **Obsługa paczek wielolinijkowych**: Buforowane wykonywanie bloków procedury Framing z LaserCAM Studio.
- **REST API**: Endpoint `/api/files/local` dla bezpośredniego wysyłania plików G-code prosto z programu CAD.

## Sterowanie zadaniem (v2)
- **Pauza** (`{"cmd":"pause"}`, `POST /api/job/pause`): feed hold `!`, zadanie zostaje w pamięci.
- **Wznów** (`{"cmd":"resume"}`, `POST /api/job/resume`): `~`, tylko po pauzie lub z `Hold`/`Door` – zablokowane po anulowaniu.
- **Anuluj i wyczyść** (`{"cmd":"cancel"}`, `POST /api/job/cancel`): `!` → czekanie na `Hold:0` → soft-reset `Ctrl-X`.
  Reset czyści bufor planisty i RX GRBL (przyjęte już linie nie zostaną wykonane) bez utraty pozycji.
  Działa też bez aktywnego zadania – np. po wciśnięciu przycisku bezpieczeństwa.
- **Odblokuj** (`{"cmd":"unlock"}`, `POST /api/unlock`): `$X` po alarmie.
- Ręczne komendy G-code są odrzucane w trakcie zadania; nowe zadanie przy trwającym zwraca `409`.
- Błąd `error:` z GRBL w trakcie zadania zatrzymuje je bezpiecznie (`job_state = "error"`).
- Stan (`/api/status`, WebSocket) zawiera `job_state`, `alarm`, `last_error`, `bridge_version`, `capabilities`.
- Protokół tekstowy v1 działa nadal; `!` oznacza w nim bezpieczne anulowanie.

Logika sterowania jest w `grbl_core.py` (bez zależności od FastAPI), serwer w `grbl_bridge.py`.

## Wdrażanie i aktualizacja na Raspberry Pi
```bash
cd /home/cnc/grbl-bridge
git pull
sudo systemctl restart grbl-bridge