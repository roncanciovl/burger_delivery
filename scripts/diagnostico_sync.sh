#!/usr/bin/env bash
# Sincronización con la base oficial del curso.
#
# Uso:
#   bash scripts/diagnostico_sync.sh                # revisa el setup e informa (no modifica nada)
#   bash scripts/diagnostico_sync.sh --configurar   # además configura el remoto upstream
#   bash scripts/diagnostico_sync.sh --sincronizar  # revisa y, si todo está bien, actualiza tu main

set -uo pipefail

UPSTREAM_URL_SSH="git@github.com:roncanciovl/burger_delivery.git"
RAMA_BASE="base-latest"
MODO="revisar"
case "${1:-}" in
  --configurar)  MODO="configurar" ;;
  --sincronizar) MODO="sincronizar" ;;
  "")            ;;
  *) echo "Opción desconocida: $1"; echo "Usa: --configurar | --sincronizar"; exit 2 ;;
esac

if [[ -t 1 ]]; then
  ROJO=$'\e[31m'; VERDE=$'\e[32m'; AMAR=$'\e[33m'; NEG=$'\e[1m'; FIN=$'\e[0m'
else
  ROJO=""; VERDE=""; AMAR=""; NEG=""; FIN=""
fi

ERRORES=()
AL_DIA=0
VERSION="(desconocida)"
ok()    { echo "  ${VERDE}OK${FIN}      $1"; }
falla() { echo "  ${ROJO}FALLA${FIN}   $1"; ERRORES+=("$2"); }
aviso() { echo "  ${AMAR}AVISO${FIN}   $1"; }
titulo(){ echo; echo "${NEG}$1${FIN}"; }

echo "${NEG}=== Sincronización con la base del curso ===${FIN}"

# ---------------------------------------------------------------- repositorio
titulo "1. Repositorio"
if ! RAIZ=$(git rev-parse --show-toplevel 2>/dev/null); then
  echo "  ${ROJO}FALLA${FIN}   No estás dentro de un repositorio Git."
  echo
  echo "  Solución:  cd ~/ros2_ws/src/burger_delivery"
  exit 1
fi
cd "$RAIZ" || exit 1
ok "Carpeta: $RAIZ"
ok "Rama actual: $(git branch --show-current 2>/dev/null || echo '(sin rama)')"

# ------------------------------------------------------------ cambios locales
titulo "2. Trabajo sin confirmar"
if [[ -z "$(git status --porcelain)" ]]; then
  ok "No hay cambios pendientes."
else
  falla "Tienes archivos modificados sin confirmar:" \
        "Confirma o guarda tu trabajo antes de sincronizar:
     git switch -c feat/apellido-avance && git add . && git commit -m 'wip: avance'
     ...o guárdalo temporalmente con:  git stash push -m 'antes de sincronizar'"
  git status --short | sed 's/^/          /'
fi

# -------------------------------------------------------------------- remotos
titulo "3. Remotos"
if git remote get-url origin >/dev/null 2>&1; then
  ok "origin  -> $(git remote get-url origin)"
else
  falla "No existe el remoto 'origin'." "Vuelve a clonar el repositorio de tu equipo."
fi

if git remote get-url upstream >/dev/null 2>&1; then
  ok "upstream -> $(git remote get-url upstream)"
  PUSH_UP=$(git remote get-url --push upstream 2>/dev/null)
  if [[ "$PUSH_UP" == "DISABLED" ]]; then
    ok "upstream protegido contra escritura (push DISABLED)."
  elif [[ "$MODO" == "configurar" ]]; then
    git remote set-url --push upstream DISABLED && ok "Protección de escritura aplicada a upstream."
  else
    falla "upstream permite escritura (push = $PUSH_UP)." \
          "Protégelo:  git remote set-url --push upstream DISABLED"
  fi
elif [[ "$MODO" == "configurar" ]]; then
  git remote add upstream "$UPSTREAM_URL_SSH" &&
  git remote set-url --push upstream DISABLED &&
  ok "Remoto upstream agregado y protegido."
else
  falla "No existe el remoto 'upstream' (repositorio del docente)." \
        "Configúralo automáticamente:  bash scripts/diagnostico_sync.sh --configurar"
fi

# ------------------------------------------------------------------ identidad
titulo "4. Identidad de tus commits"
NOMBRE=$(git config user.name || true)
CORREO=$(git config user.email || true)
if [[ -n "$NOMBRE" && -n "$CORREO" ]]; then
  ok "$NOMBRE <$CORREO>"
else
  falla "Git no sabe quién eres; tus commits no servirán como evidencia de autoría." \
        "Configúrala:
     git config user.name  'Nombres y Apellidos'
     git config user.email 'codigo.estudiante@unimilitar.edu.co'"
fi

# ------------------------------------------------------------------- conexión
titulo "5. Conexión con GitHub"
if git ls-remote --heads origin >/dev/null 2>&1; then
  ok "Acceso al repositorio de tu equipo confirmado."
else
  falla "No se pudo acceder a 'origin' (credenciales o red)." \
        "Verifica tu llave SSH:  ssh -T git@github.com
     Guía: git-fundamentals/ssh_keys.html"
fi

# --------------------------------------------------- base publicada y novedades
titulo "6. Base publicada por el docente"
if git remote get-url upstream >/dev/null 2>&1; then
  git fetch --quiet upstream --tags 2>/dev/null
  git fetch --quiet origin 2>/dev/null
  if git rev-parse -q --verify "refs/remotes/upstream/${RAMA_BASE}" >/dev/null; then
    VERSION=$(git describe --tags --always "upstream/${RAMA_BASE}" 2>/dev/null)
    ok "Rama upstream/${RAMA_BASE} disponible — versión: ${NEG}${VERSION}${FIN}"
    NUEVOS=$(git rev-list --count "origin/main..upstream/${RAMA_BASE}" 2>/dev/null || echo 0)
    if [[ "$NUEVOS" == "0" ]]; then
      ok "Tu equipo ya está al día: no hay nada que sincronizar."
      AL_DIA=1
    else
      aviso "Hay ${NUEVOS} commit(s) nuevos por integrar:"
      git log --oneline "origin/main..upstream/${RAMA_BASE}" | head -10 | sed 's/^/          /'
      echo "          ---"
      git diff --stat "origin/main..upstream/${RAMA_BASE}" | tail -1 | sed 's/^/          /'
    fi
  else
    falla "No se encontró la rama upstream/${RAMA_BASE}." \
          "Reintenta la descarga:  git fetch upstream
     Si persiste, avisa al docente: la rama de publicación no está disponible."
  fi
else
  aviso "Se omite: falta configurar el remoto upstream."
fi

# ------------------------------------------------------- errores: se detiene aquí
if [[ ${#ERRORES[@]} -gt 0 ]]; then
  echo
  echo "${ROJO}${NEG}CORRIGE ESTO ANTES DE SINCRONIZAR (${#ERRORES[@]}):${FIN}"
  n=1
  for e in "${ERRORES[@]}"; do
    echo
    echo "  ${NEG}${n}.${FIN} ${e}"
    n=$((n + 1))
  done
  echo
  exit 1
fi

# --------------------------------------------------------------- ya está al día
if [[ $AL_DIA -eq 1 ]]; then
  echo
  echo "${VERDE}${NEG}NADA QUE HACER.${FIN} Tu equipo ya tiene la última base publicada."
  echo "  Vuelve a ejecutar esto cuando el docente anuncie material nuevo."
  exit 0
fi

# ------------------------------------------------------------- modo sólo revisar
if [[ "$MODO" != "sincronizar" ]]; then
  echo
  echo "${VERDE}${NEG}TODO EN ORDEN.${FIN} Para integrar la base ejecuta:"
  echo
  echo "    bash scripts/diagnostico_sync.sh --sincronizar"
  echo
  exit 0
fi

# ------------------------------------------------------------------ sincronizar
titulo "7. Integrando la base en tu rama main"

if ! git switch --quiet main 2>/dev/null; then
  echo "  ${ROJO}FALLA${FIN}   No se pudo cambiar a la rama main."
  exit 1
fi
ok "En la rama main."

if ! git pull --quiet --ff-only origin main 2>/dev/null; then
  echo "  ${ROJO}FALLA${FIN}   Tu main local tiene commits que no están en GitHub."
  echo
  echo "  Muévelos a una rama propia y vuelve a intentarlo:"
  echo "     git switch -c feat/apellido-avance"
  echo "     git push -u origin feat/apellido-avance"
  exit 1
fi
ok "main actualizada desde GitHub."

if git merge --no-ff -m "chore: sincronizar base oficial del curso (${VERSION})" "upstream/${RAMA_BASE}"; then
  ok "Base integrada sin conflictos."
else
  echo
  echo "${AMAR}${NEG}HAY CONFLICTOS.${FIN} El docente y tu equipo editaron las mismas líneas."
  echo "  Archivos por resolver:"
  git diff --name-only --diff-filter=U | sed 's/^/     /'
  echo
  echo "  Termina así:"
  echo "     1. Abre cada archivo y deja la versión correcta."
  echo "        Borra las marcas  <<<<<<<   =======   >>>>>>>"
  echo "     2. git add <archivos>"
  echo "     3. git commit"
  echo "     4. git push origin main"
  echo
  echo "  ¿Prefieres cancelar y volver al estado anterior?   git merge --abort"
  exit 1
fi

echo
read -r -p "¿Publicar el resultado en GitHub (git push origin main)? [s/N] " RESP
if [[ "$RESP" =~ ^[sSyY]$ ]]; then
  if git push origin main; then
    echo
    echo "${VERDE}${NEG}LISTO.${FIN} Tu equipo quedó con la base ${VERSION}."
    echo "  Avisa a tus compañeros: cada uno debe ejecutar 'git pull --ff-only origin main'."
  else
    echo
    echo "${ROJO}${NEG}El push fue rechazado.${FIN}"
    echo "  Si el repositorio exige Pull Request, publica el resultado en una rama:"
    echo "     git switch -c \"sync/base-\$(date +%Y%m%d)\""
    echo "     git push -u origin HEAD"
    exit 1
  fi
else
  echo
  echo "  El merge quedó hecho sólo en tu computador. Cuando quieras publicarlo:"
  echo "     git push origin main"
fi
