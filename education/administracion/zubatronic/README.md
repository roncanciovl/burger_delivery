# Preparación de carga para Zubatronic/SGDE

Esta carpeta contiene una automatización local y auditable para convertir los
instrumentos oficiales de **ELECTIVA ELECTRONICA ( ROBOT OPERATING SYSTEM** en una vista previa
de captura compatible con los campos actuales de Zubatronic/SGDE.

El programa no escribe en Firebase ni controla el navegador. Su salida sirve
como contrato antes de aplicar cambios mediante la interfaz docente.

## Generar la vista previa

Desde la raíz de `education`:

```bash
python3 administracion/zubatronic/preparar_carga_ros.py
```

Para generar también el CSV estudiantil privado:

```bash
python3 administracion/zubatronic/preparar_carga_ros.py \
  --student-xlsx "/ruta/privada/Lista de Estudiantes ROS.xlsx" \
  --student-output "/ruta/privada/Lista_Estudiantes_ROS_Zubatronic.csv"
```

La lista contiene datos personales y debe permanecer fuera del repositorio.

## Comparar con la pestaña SGDE

Con Chrome abierto, autenticado y disponible en el puerto CDP configurado:

```bash
python3 administracion/zubatronic/sincronizar_zubatronic.py \
  --student-csv "/ruta/privada/Lista_Estudiantes_ROS_Zubatronic.csv"
```

El comando sólo presenta la diferencia. La aplicación requiere además
`--apply --confirm APLICAR`, se bloquea si faltan RAE y nunca elimina criterios,
estudiantes, instrumentos ni vínculos externos.

## Salidas públicas

`preview/` contiene:

- `ros_corte1_zubatronic.preview.json`: tres instrumentos, quince criterios,
  pesos, RAE/SO y descriptores N1–N5;
- `rae_ros_2026-2_admin.csv`: veintiún RAE para carga por un administrador;
- `RESUMEN_PREVIO_CARGA_ROS.md`: revisión humana antes de aplicar.

## Estado y restricción actual

El curso SGDE `ELECTIVA ELECTRONICA (  ROBOT OPERATING SYSTEM)|A|2026-2` ya
tiene los instrumentos 28% + 42% + 30%, pero no tiene criterios, estudiantes ni
RAE disponibles. Primero un administrador debe cargar el CSV de RAE. Después se
pueden ingresar estudiantes y criterios mediante la interfaz, siempre con una
lectura de verificación posterior.

No se debe usar **Crear en Classroom**: el curso de Classroom ya fue creado por
el flujo de Apps Script y SGDE no ofrece una operación para vincularlo.
