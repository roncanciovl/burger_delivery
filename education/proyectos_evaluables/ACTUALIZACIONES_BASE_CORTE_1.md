# 🔄 Sincronización con la base oficial del curso — 2026-2

> **Asignatura:** ELECTIVA ELECTRONICA ( ROBOT OPERATING SYSTEM ) · **Periodo:** 2026-2  
> **Repositorio docente (`upstream`):** [`roncanciovl/burger_delivery`](https://github.com/roncanciovl/burger_delivery) · **Organización:** [`umng-mecatronica-ros`](https://github.com/umng-mecatronica-ros)

Cuando el docente publica material nuevo, tu equipo lo incorpora con **dos comandos**. Funcionan igual todo el semestre: siempre traen la última versión oficial desde la rama `upstream/base-latest`.

---

## 0. Arranque único — sólo si tu equipo no tiene el script

Si el punto 1 responde `bash: scripts/diagnostico_sync.sh: No such file or directory`, la base de tu equipo es anterior a la publicación del script. Haz **una sola vez** esa misma integración a mano: el script viene dentro de la base, así que después sigues el flujo normal.

```bash
cd ~/ros2_ws/src/burger_delivery
git remote add upstream git@github.com:roncanciovl/burger_delivery.git
git remote set-url --push upstream DISABLED
git fetch upstream
git switch main
git pull --ff-only origin main
git merge --no-ff upstream/base-latest
git push origin main
```

Si `git remote add` responde `remote upstream already exists`, sáltate esa línea y sigue con el resto. Los conflictos y el push rechazado se resuelven igual que en el punto 2.

Confirma que quedó listo:

```bash
bash scripts/diagnostico_sync.sh
```

Debe responder `NADA QUE HACER`. A partir de aquí no vuelves a usar este punto.

---

## 1. Revisar qué hay pendiente

```bash
cd ~/ros2_ws/src/burger_delivery
bash scripts/diagnostico_sync.sh
```

No modifica nada. Revisa tu setup (carpeta, trabajo sin confirmar, remotos, identidad, acceso a GitHub) y te muestra qué publicó el docente. Termina en uno de tres veredictos:

| Veredicto | Qué hacer |
|---|---|
| `TODO EN ORDEN` | Continúa al punto 2. |
| `NADA QUE HACER` | Ya tienes la última base. No hagas nada más. |
| `CORRIGE ESTO ANTES DE SINCRONIZAR` | El script lista cada problema **con el comando que lo soluciona**. Corrígelos y vuelve a ejecutarlo. |

> **La primera vez**, si aún no tienes configurado el repositorio del docente, deja que el script lo haga por ti:
> ```bash
> bash scripts/diagnostico_sync.sh --configurar
> ```

---

## 2. Integrar la base

```bash
bash scripts/diagnostico_sync.sh --sincronizar
```

Vuelve a revisar todo y, si está en orden, actualiza tu `main`, fusiona la base del docente y te pregunta si quieres publicarla en GitHub. **Cada integrante puede ejecutarlo por su cuenta**; quien lo haga primero publica, y a los demás el script les dirá que ya están al día.

**Si aparecen conflictos**, el script se detiene y te dice qué archivos revisar. Significa que el docente y tu equipo editaron las mismas líneas:

1. Abre cada archivo listado y busca los bloques `<<<<<<<` / `=======` / `>>>>>>>`.
2. Deja la versión correcta y **borra esas marcas**.
3. Cierra el merge:
   ```bash
   git add <archivos>
   git commit
   git push origin main
   ```

¿Prefieres empezar de cero? `git merge --abort` deshace todo y te deja como estabas.

---

## 3. Los demás integrantes actualizan su copia

Cuando alguien del equipo publique la sincronización:

```bash
git switch main
git pull --ff-only origin main
```

---

> [!IMPORTANT]
> **Esto aplica sólo a la base del docente.** El trabajo propio del equipo sigue entrando por rama y Pull Request revisado por un compañero (`git switch -c feat/apellido-tema`), porque esa revisión es evidencia evaluable del curso. Sincronizar es una operación mecánica sobre material ajeno; desarrollar no.

---

## Problemas frecuentes

| Síntoma | Solución |
|---|---|
| `fatal: not a git repository` | Estás en otra carpeta: `cd ~/ros2_ws/src/burger_delivery` |
| `scripts/diagnostico_sync.sh: No such file or directory` | Tu base es anterior al script: haz el **punto 0** una vez. |
| Tienes cambios sin confirmar | `git stash push -m "antes de sincronizar"` (los recuperas con `git stash pop`), o consérvalos en una rama propia con `git switch -c feat/apellido-avance` y un commit. |
| `Permission denied (publickey)` | Tu llave SSH no está en GitHub. Verifica con `ssh -T git@github.com` y revisa la [guía de llaves SSH](../../git-fundamentals/ssh_keys.html). |
| Tu `main` local tiene commits propios | Muévelos a una rama: `git switch -c feat/apellido-avance` y `git push -u origin HEAD`. |
| El push a `main` fue rechazado | El repositorio exige Pull Request. Publica en una rama: `git switch -c "sync/base-$(date +%Y%m%d)"` y `git push -u origin HEAD`. |
| `upstream/base-latest` no existe | `git fetch upstream`. Si persiste, avisa al docente. |
| Te arrepentiste del merge | `git merge --abort` (antes del push) o `git reset --hard origin/main` si aún no habías publicado. |

---

## Preguntas frecuentes

**¿Debo cambiar algún comando cuando el docente publique material nuevo?**  
No. `upstream/base-latest` avanza sola; repites los mismos dos comandos las veces que haga falta.

**¿Por qué no hay un tag `last` que se reasigne?**  
Porque Git no actualiza un tag ya descargado salvo con `--force`: dos equipos podrían creer que tienen la misma versión y tener contenidos distintos. Las ramas están hechas para avanzar y los tags para quedarse quietos. Cada publicación queda marcada con un tag inmutable con fecha (`base-2026-2-AAAAMMDD`).

**¿Queda evidencia de qué versión integré?**  
Sí. El commit de merge guarda el SHA exacto del estado que integraste, con fecha y autor, y su mensaje incluye el nombre del tag inmutable correspondiente.

**¿Perderé el trabajo de mi equipo al fusionar?**  
No. El merge **agrega** los cambios del docente sobre los tuyos; sólo te pide decidir cuando ambos tocaron las mismas líneas. Y el script no te deja empezar si tienes trabajo sin confirmar.

**¿Por qué no `git pull upstream main`?**  
Porque `upstream/main` puede contener trabajo en curso del docente. Sólo `base-latest` está avalada para consumo de los equipos.

**¿Varios computadores pueden ejecutar el driver del robot (`kortex_driver`) a la vez?**  
**No.** La controladora del Kinova Gen3 admite una sola sesión cíclica en tiempo real (múltiples conexiones disparan *Safety Faults*) y se duplicarían `/joint_states`, `/tf` y `/controller_manager`. La estación conectada al robot usa `start_driver:=true robot_ip:=192.168.1.10`; las demás, `start_driver:=false`. En modo `use_fake_hardware:=true`, cada equipo usa un `ROS_DOMAIN_ID` distinto.

**¿Cómo se congela la entrega?**
```bash
git switch main && git pull --ff-only origin main
git tag -a entrega-2026-2-corte1 -m "Entrega oficial 2026-2 - corte 1"
git push origin entrega-2026-2-corte1
git rev-parse HEAD
```
El SHA resultante es el identificador definitivo registrado en la evaluación.

---

## Anexo — Publicación de la base (uso del docente)

`main` puede contener trabajo en curso; sólo lo que llega a `base-latest` queda disponible para los equipos:

```bash
git push origin main:base-latest
```

Ese push dispara [`.github/workflows/publicar-base.yml`](../../.github/workflows/publicar-base.yml), que crea un tag anotado inmutable `base-2026-2-AAAAMMDD` (con sufijo `.2`, `.3`… si hay varias publicaciones el mismo día) y un GitHub Release marcado *latest*.

* El workflow sólo corre en `roncanciovl/burger_delivery`; en los repositorios de los equipos, que heredan el archivo, queda desactivado por la condición `if: github.repository == ...`.
* El periodo académico se actualiza una vez por semestre en la variable `PERIODO` del workflow.
* Para reestampar sin mover la rama: **Actions → Publicar base del curso → Run workflow**.
* Como los equipos fusionan la base directamente en su `main`, la protección de rama de sus repositorios no debe exigir Pull Request para `main`; la disciplina de PR se mantiene para el trabajo propio de cada equipo.
