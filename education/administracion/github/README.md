# Sincronización de equipos con GitHub

Esta carpeta contiene el puente local entre la hoja privada `Equipos` del panel
de Classroom y la organización `umng-mecatronica-ros`. El panel es la fuente
única de conformación; GitHub recibe una réplica controlada.

## Preparación de la hoja

Complete para cada estudiante:

- `EQUIPO_ID`: `equipo-01` a `equipo-09`; `equipo-10` queda de reserva;
- `GITHUB_USER`: nombre exacto de la cuenta personal;
- `ROL_INICIAL`: desarrollo, integración o pruebas/documentación;
- `ENTREGANTE`: exactamente una persona por equipo;
- `REPOSITORIO` y `GITHUB_TEAM` se calculan automáticamente.

Cada equipo activo debe tener dos o tres integrantes. Para 19 estudiantes se
recomiendan ocho parejas y un trío.

## Exportación privada

1. Abra el panel privado de Google Sheets.
2. Entre en la pestaña `Equipos`.
3. Use **Archivo > Descargar > Valores separados por comas (.csv)**.
4. Guarde el archivo dentro de un directorio local `datos_privados/` en esta
   carpeta. Ese directorio y los CSV están excluidos de Git.

No publique el CSV: contiene datos personales y la correspondencia entre
identidades institucionales y cuentas de GitHub.

## Vista previa

```bash
cd ~/ros2_ws/src/burger_delivery
python3 education/administracion/github/sincronizar_equipos.py \
  --csv education/administracion/github/datos_privados/Equipos.csv
```

La vista previa valida el CSV, las cuentas, los GitHub Teams y los repositorios
privados. También informa accesos adicionales, pero nunca los retira.

## Aplicación

La cuenta autenticada debe ser administradora de la organización. Si el token
actual de `gh` no permite administrar integrantes, amplíe su autorización:

```bash
gh auth refresh -h github.com -s admin:org
```

Después de revisar la vista previa:

```bash
python3 education/administracion/github/sincronizar_equipos.py \
  --csv education/administracion/github/datos_privados/Equipos.csv \
  --modo APLICAR \
  --confirmar umng-mecatronica-ros/2026-2
```

El modo `APLICAR` confirma permiso `push` del Team sobre su repositorio e
incorpora cada cuenta como `member`. Si la persona aún no pertenece a la
organización, GitHub deja la membresía pendiente hasta que acepte la invitación.

La herramienta es aditiva: no elimina integrantes, repositorios, Teams ni
permisos. Cualquier retiro debe revisarse y ejecutarse separadamente.
