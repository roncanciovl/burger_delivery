/**
 * CONTROLADOR CLASSROOM — ROBOT OPERATING SYSTEM - ROS
 * Periodo 2026-2
 *
 * Se instala una sola vez en Google Apps Script. La configuración variable,
 * los estudiantes, las actividades y los resultados se conservan en una hoja
 * privada creada por instalarControladorROS().
 *
 * Requiere el servicio avanzado "Google Classroom API".
 */

var ROS_CTRL = {
  version: "1.1.0",
  panelProperty: "ROS_CLASSROOM_PANEL_ID",
  courseProperty: "ROS_2026_2_COURSE_ID",
  timeZone: "America/Bogota",
  studentDomain: "unimilitar.edu.co",
  sheets: {
    config: "Configuracion",
    students: "Estudiantes",
    teams: "Equipos",
    activities: "Actividades",
    submissions: "Entregas",
    audit: "Auditoria"
  }
};

function onOpen() {
  SpreadsheetApp.getUi()
    .createMenu("ROS — Administración")
    .addItem("Actualizar estructura del panel", "instalarControladorROS")
    .addItem("Sincronizar estudiantes en Equipos", "sincronizarEquiposDesdeEstudiantesROS")
    .addSeparator()
    .addItem("Vista previa de Classroom", "vistaPreviaControladorROS")
    .addItem("Vista previa de equipos", "vistaPreviaEquiposROS")
    .addItem("Aplicar configuración de Classroom", "configurarTodoROS")
    .addSeparator()
    .addItem("Sincronizar entregas", "sincronizarEntregasROS")
    .addItem("Auditar curso", "auditarControladorROS")
    .addToUi();
}

/**
 * Crea una vez el panel privado de administración y devuelve su enlace.
 * Si ya existe, lo reutiliza sin borrar información.
 */
function instalarControladorROS() {
  var panel = obtenerPanelSiExiste_();
  var creado = false;

  if (!panel) {
    panel = SpreadsheetApp.create("PANEL CLASSROOM - ROBOT OPERATING SYSTEM - ROS - 2026-2");
    PropertiesService.getUserProperties().setProperty(
      ROS_CTRL.panelProperty,
      panel.getId()
    );
    creado = true;
  }

  panel.setSpreadsheetTimeZone(ROS_CTRL.timeZone);
  inicializarPanel_(panel);

  var resultado = {
    ok: true,
    creado: creado,
    version: ROS_CTRL.version,
    mensaje: creado
      ? "Panel privado creado correctamente."
      : "El panel ya existía y fue reutilizado.",
    enlacePanel: panel.getUrl()
  };
  registrarAuditoria_(panel, "INSTALAR_CONTROLADOR", resultado);
  console.log(JSON.stringify(resultado, null, 2));
  return resultado;
}

/**
 * Prueba mínima de lectura. No crea ni modifica recursos de Classroom.
 */
function probarAccesoClassroom() {
  try {
    var respuesta = Classroom.Courses.list({ pageSize: 10 });
    var cursos = respuesta.courses || [];
    var resultado = {
      ok: true,
      mensaje: "La cuenta puede consultar Google Classroom.",
      cursosEncontrados: cursos.length,
      cursos: cursos.map(function(curso) {
        return resumenCurso_(curso);
      })
    };
    console.log(JSON.stringify(resultado, null, 2));
    return resultado;
  } catch (error) {
    var resultadoError = resultadoError_(
      "No fue posible consultar Google Classroom.",
      error
    );
    console.error(JSON.stringify(resultadoError, null, 2));
    return resultadoError;
  }
}

/**
 * Lee el panel, valida la información y muestra las acciones previstas.
 * No cambia Classroom.
 */
function vistaPreviaControladorROS() {
  try {
    var panel = obtenerPanel_();
    var config = leerConfiguracion_(panel);
    var estudiantes = leerEstudiantes_(panel, config);
    var equipos = leerEquipos_(panel);
    var actividades = leerActividades_(panel);
    var curso = localizarCurso_(config);
    var resumenEstudiantes = validarEstudiantes_(estudiantes, config);
    var resumenActividades = validarActividades_(actividades);
    var requiereEquipos = actividades.some(function(actividad) {
      return actividad.activa && actividad.asignacion === "ENTREGANTES";
    });
    var resumenEquipos = validarEquiposClassroom_(equipos, estudiantes);

    var resultado = {
      ok:
        resumenEstudiantes.ok &&
        resumenActividades.ok &&
        (!requiereEquipos || resumenEquipos.ok),
      version: ROS_CTRL.version,
      modo: config.MODO,
      curso: curso
        ? { accion: "REUTILIZAR", datos: resumenCurso_(curso) }
        : {
            accion: "CREAR",
            nombre: config.NOMBRE_CURSO,
            seccion: config.SECCION,
            estado: config.ESTADO_CURSO
          },
      estudiantes: resumenEstudiantes,
      equipos: resumenEquipos,
      actividades: resumenActividades,
      advertencia:
        "Esta función es de solo lectura. Para aplicar cambios escriba APLICAR en MODO y ejecute configurarTodoROS()."
    };

    registrarAuditoria_(panel, "VISTA_PREVIA", resultado);
    console.log(JSON.stringify(resultado, null, 2));
    return resultado;
  } catch (error) {
    var resultadoError = resultadoError_("Falló la vista previa.", error);
    console.error(JSON.stringify(resultadoError, null, 2));
    return resultadoError;
  }
}

/**
 * Configuración inicial en una sola ejecución e idempotente:
 * 1. crea o reutiliza el curso;
 * 2. crea los temas faltantes;
 * 3. envía las invitaciones faltantes;
 * 4. crea como BORRADOR las actividades faltantes;
 * 5. registra una auditoría.
 *
 * Requiere MODO=APLICAR en la hoja Configuracion. Al terminar, el modo vuelve
 * automáticamente a VISTA_PREVIA.
 */
function configurarTodoROS() {
  var panel = obtenerPanel_();
  var config = leerConfiguracion_(panel);
  exigirModoAplicar_(config);

  try {
    var estudiantes = leerEstudiantes_(panel, config);
    var equipos = leerEquipos_(panel);
    var actividades = leerActividades_(panel);
    var validacionEstudiantes = validarEstudiantes_(estudiantes, config);
    var validacionActividades = validarActividades_(actividades);
    var requiereEquipos = actividades.some(function(actividad) {
      return actividad.activa && actividad.asignacion === "ENTREGANTES";
    });
    var validacionEquipos = validarEquiposClassroom_(equipos, estudiantes);

    if (!validacionEstudiantes.ok) {
      throw new Error(
        "La lista de estudiantes tiene incidencias. Corríjalas antes de aplicar: " +
          validacionEstudiantes.incidencias.join(" | ")
      );
    }
    if (!validacionActividades.ok) {
      throw new Error(
        "La tabla de actividades tiene incidencias. Corríjalas antes de aplicar: " +
          validacionActividades.incidencias.join(" | ")
      );
    }
    if (requiereEquipos && !validacionEquipos.ok) {
      throw new Error(
        "Las actividades grupales requieren equipos completos: " +
          validacionEquipos.incidencias.join(" | ")
      );
    }

    var curso = crearOReutilizarCurso_(panel, config);
    var temas = crearOReutilizarTemas_(curso.id, actividades);
    var invitaciones = enviarInvitaciones_(panel, curso.id, estudiantes);
    var borradores = crearBorradores_(panel, curso.id, actividades, temas);
    var auditoria = auditarEstado_(curso.id);

    var resultado = {
      ok: true,
      mensaje: "Configuración inicial terminada.",
      curso: resumenCurso_(curso),
      temas: temas.resumen,
      invitaciones: invitaciones,
      borradores: borradores,
      estadoFinal: auditoria
    };
    registrarAuditoria_(panel, "CONFIGURAR_TODO", resultado);
    console.log(JSON.stringify(resultado, null, 2));
    return resultado;
  } catch (error) {
    var resultadoError = resultadoError_("Falló la configuración inicial.", error);
    registrarAuditoria_(panel, "ERROR_CONFIGURAR_TODO", resultadoError);
    console.error(JSON.stringify(resultadoError, null, 2));
    return resultadoError;
  } finally {
    escribirConfiguracion_(panel, "MODO", "VISTA_PREVIA");
  }
}

/**
 * Genera un inventario privado de entregas y adjuntos en la hoja Entregas.
 * No modifica Classroom ni asigna calificaciones.
 */
function sincronizarEntregasROS() {
  try {
    var panel = obtenerPanel_();
    var config = leerConfiguracion_(panel);
    var courseId = obtenerCourseId_(config);
    var actividades = leerActividades_(panel).filter(function(actividad) {
      return actividad.idClassroom;
    });
    var perfiles = obtenerMapaPerfiles_(courseId);
    var filas = [];

    actividades.forEach(function(actividad) {
      var entregas = listarEntregas_(courseId, actividad.idClassroom);
      entregas.forEach(function(entrega) {
        var perfil = perfiles[entrega.userId] || {};
        var adjuntos = extraerAdjuntos_(entrega);
        filas.push([
          new Date(),
          actividad.codigo,
          actividad.titulo,
          entrega.userId || "",
          perfil.email || "",
          perfil.nombre || "",
          entrega.state || "",
          entrega.late === true ? "SI" : "NO",
          valorSeguro_(entrega.draftGrade),
          valorSeguro_(entrega.assignedGrade),
          entrega.updateTime || "",
          adjuntos.titulos.join(" | "),
          adjuntos.enlaces.join(" | ")
        ]);
      });
    });

    var hoja = panel.getSheetByName(ROS_CTRL.sheets.submissions);
    var encabezados = [[
      "SINCRONIZADO_EN",
      "ACTIVIDAD",
      "TITULO",
      "USER_ID",
      "CORREO",
      "NOMBRE",
      "ESTADO_ENTREGA",
      "TARDE",
      "NOTA_BORRADOR",
      "NOTA_ASIGNADA",
      "ACTUALIZADO_EN",
      "ADJUNTOS",
      "ENLACES"
    ]];
    hoja.clearContents();
    hoja.getRange(1, 1, 1, encabezados[0].length).setValues(encabezados);
    if (filas.length) {
      hoja.getRange(2, 1, filas.length, encabezados[0].length).setValues(filas);
    }
    formatearHoja_(hoja, encabezados[0].length);

    var resultado = {
      ok: true,
      actividadesRevisadas: actividades.length,
      entregasInventariadas: filas.length,
      mensaje:
        "Inventario actualizado. La selección de niveles y la calificación continúan siendo decisiones docentes."
    };
    registrarAuditoria_(panel, "SINCRONIZAR_ENTREGAS", resultado);
    console.log(JSON.stringify(resultado, null, 2));
    return resultado;
  } catch (error) {
    var resultadoError = resultadoError_("No se pudo sincronizar el inventario.", error);
    console.error(JSON.stringify(resultadoError, null, 2));
    return resultadoError;
  }
}

/**
 * Consulta el estado actual del curso sin modificar Classroom.
 */
function auditarControladorROS() {
  try {
    var panel = obtenerPanel_();
    var config = leerConfiguracion_(panel);
    var courseId = obtenerCourseId_(config);
    var estado = auditarEstado_(courseId);
    var resultado = { ok: true, estado: estado };
    registrarAuditoria_(panel, "AUDITAR_CURSO", resultado);
    console.log(JSON.stringify(resultado, null, 2));
    return resultado;
  } catch (error) {
    var resultadoError = resultadoError_("No fue posible auditar el curso.", error);
    console.error(JSON.stringify(resultadoError, null, 2));
    return resultadoError;
  }
}

/**
 * Incorpora en Equipos a los estudiantes faltantes sin sobrescribir la
 * conformación, el usuario de GitHub, los roles ni los estados existentes.
 */
function sincronizarEquiposDesdeEstudiantesROS() {
  try {
    var panel = obtenerPanel_();
    var config = leerConfiguracion_(panel);
    var estudiantes = leerEstudiantes_(panel, config);
    var resultado = sincronizarEstudiantesEnEquipos_(panel, estudiantes);
    registrarAuditoria_(panel, "SINCRONIZAR_EQUIPOS", resultado);
    console.log(JSON.stringify(resultado, null, 2));
    return resultado;
  } catch (error) {
    var resultadoError = resultadoError_("No fue posible sincronizar Equipos.", error);
    console.error(JSON.stringify(resultadoError, null, 2));
    return resultadoError;
  }
}

/**
 * Valida la fuente única de equipos. No modifica Classroom ni GitHub.
 */
function vistaPreviaEquiposROS() {
  try {
    var panel = obtenerPanel_();
    var config = leerConfiguracion_(panel);
    var estudiantes = leerEstudiantes_(panel, config);
    var equipos = leerEquipos_(panel);
    var classroom = validarEquiposClassroom_(equipos, estudiantes);
    var github = validarEquiposGithub_(equipos, estudiantes);
    var resultado = {
      ok: classroom.ok && github.ok,
      classroom: classroom,
      github: github,
      advertencia:
        "Esta función sólo valida. GitHub se sincroniza localmente con gh después de exportar la hoja Equipos como CSV."
    };
    registrarAuditoria_(panel, "VISTA_PREVIA_EQUIPOS", resultado);
    console.log(JSON.stringify(resultado, null, 2));
    return resultado;
  } catch (error) {
    var resultadoError = resultadoError_("Falló la vista previa de equipos.", error);
    console.error(JSON.stringify(resultadoError, null, 2));
    return resultadoError;
  }
}

function inicializarPanel_(panel) {
  var config = asegurarHoja_(panel, ROS_CTRL.sheets.config);
  var students = asegurarHoja_(panel, ROS_CTRL.sheets.students);
  var teams = asegurarHoja_(panel, ROS_CTRL.sheets.teams);
  var activities = asegurarHoja_(panel, ROS_CTRL.sheets.activities);
  var submissions = asegurarHoja_(panel, ROS_CTRL.sheets.submissions);
  var audit = asegurarHoja_(panel, ROS_CTRL.sheets.audit);

  if (config.getLastRow() === 0) {
    var configRows = [
      ["CLAVE", "VALOR", "DESCRIPCION"],
      ["MODO", "VISTA_PREVIA", "Cambie temporalmente a APLICAR para ejecutar cambios externos."],
      ["NOMBRE_CURSO", "ROBOT OPERATING SYSTEM - ROS", "Nombre oficial de la asignatura."],
      ["SECCION", "2026-2", "Periodo académico."],
      ["ESTADO_CURSO", "ACTIVE", "Estado inicial de la clase."],
      ["DESCRIPCION_CORTA", "Curso electivo de integración y desarrollo con ROS 2", "Encabezado de Classroom."],
      ["DOMINIO_ESTUDIANTES", ROS_CTRL.studentDomain, "Dominio permitido para invitaciones."],
      ["COURSE_ID", "", "Se completa automáticamente; no editar después de crear el curso."],
      ["REPOSITORIO_URL", "https://github.com/roncanciovl/burger_delivery", "Fuente oficial del material y del código."],
      ["ZONA_HORARIA", ROS_CTRL.timeZone, "Las fechas se interpretan con hora de Bogotá."]
    ];
    config.getRange(1, 1, configRows.length, configRows[0].length).setValues(configRows);
    formatearHoja_(config, configRows[0].length);
  }

  if (students.getLastRow() === 0) {
    var studentRows = [[
      "CODIGO",
      "APELLIDOS",
      "NOMBRES",
      "CORREO",
      "ESTADO_CLASSROOM",
      "OBSERVACIONES"
    ]];
    students.getRange(1, 1, 1, studentRows[0].length).setValues(studentRows);
    students.getRange("A:D").setNumberFormat("@");
    formatearHoja_(students, studentRows[0].length);
  }

  inicializarHojaEquipos_(teams);
  sincronizarEstudiantesEnEquipos_(panel, leerEstudiantes_(panel, leerConfiguracion_(panel)));

  if (activities.getLastRow() === 0) {
    var repo = "https://github.com/roncanciovl/burger_delivery/blob/main/education/";
    var activityRows = [
      [
        "ACTIVA",
        "CODIGO",
        "TITULO",
        "TOPICO",
        "PUNTOS",
        "FECHA_LIMITE",
        "HORA_LIMITE",
        "DESCRIPCION",
        "MATERIAL_URL",
        "ID_CLASSROOM",
        "RESULTADO",
        "ASIGNACION",
        "EQUIPOS_OBJETIVO"
      ],
      [
        "SI",
        "T01",
        "Taller 01 — CLI e introspección de ROS 2",
        "Corte 1 — Fundamentos y red ROS 2",
        500,
        "",
        "",
        "Entrega individual. Use la plantilla oficial y nombre el archivo C1_T01_<codigo>_<apellido>_v1.docx.",
        repo + "talleres/TALLER_ROS2_CLI.md",
        "",
        "PENDIENTE",
        "TODOS",
        ""
      ],
      [
        "SI",
        "L01",
        "Laboratorio 01 — Evidencia común por pareja",
        "Corte 1 — Fundamentos y red ROS 2",
        500,
        "",
        "",
        "Recopilación del documento original producido durante la práctica. Una entrega por pareja; no se agregan experimentos retroactivos.",
        repo + "guias_laboratorio/rendered/GUIA_LAB_01_RED_ROS2_TALKER_LISTENER.pdf",
        "",
        "PENDIENTE",
        "TODOS",
        ""
      ],
      [
        "SI",
        "L01-IND",
        "Laboratorio 01 — Comprobación individual",
        "Corte 1 — Fundamentos y red ROS 2",
        "",
        "",
        "",
        "Comprobación individual breve de rol, diagnóstico, seguridad y contribución. No modifica la evidencia técnica común ni crea una nota académica adicional.",
        repo + "evidencias_abet/INSTRUMENTO_ABET_LAB_01_RED_ROS2_DISTRIBUIDA.docx",
        "",
        "PENDIENTE",
        "TODOS",
        ""
      ],
      [
        "NO",
        "E01",
        "Proyecto Corte 1 — Conexión segura con Kinova Gen3",
        "Corte 1 — Fundamentos y red ROS 2",
        500,
        "",
        "",
        "Proyecto integrador del primer corte. Active esta fila cuando la fecha y las instrucciones estén confirmadas.",
        repo + "proyectos_evaluables/PROYECTO_CORTE_1_CONEXION_KINOVA.md",
        "",
        "PENDIENTE",
        "ENTREGANTES",
        ""
      ]
    ];
    activities
      .getRange(1, 1, activityRows.length, activityRows[0].length)
      .setValues(activityRows);
    activities.getRange("A:M").setWrap(true);
    formatearHoja_(activities, activityRows[0].length);
  }


  asegurarColumnasActividades_(activities);

  if (submissions.getLastRow() === 0) {
    submissions.getRange(1, 1).setValue("Ejecute sincronizarEntregasROS() después de recibir entregas.");
  }

  if (audit.getLastRow() === 0) {
    audit.getRange(1, 1, 1, 4).setValues([[
      "FECHA",
      "ACCION",
      "OK",
      "RESUMEN_JSON"
    ]]);
    formatearHoja_(audit, 4);
  }

  var defaultSheet = panel.getSheetByName("Sheet1") || panel.getSheetByName("Hoja 1");
  if (defaultSheet && panel.getSheets().length > 1) {
    panel.deleteSheet(defaultSheet);
  }
}

function crearOReutilizarCurso_(panel, config) {
  var curso = localizarCurso_(config);
  if (!curso) {
    var body = {
      name: config.NOMBRE_CURSO,
      section: config.SECCION,
      descriptionHeading: config.DESCRIPCION_CORTA,
      description:
        "Espacio oficial del periodo " +
        config.SECCION +
        ". Classroom recibe actividades y entregas; GitHub conserva el código y Zubatronic consolida las rúbricas y reportes oficiales.",
      ownerId: "me",
      courseState: config.ESTADO_CURSO || "ACTIVE"
    };
    curso = Classroom.Courses.create(body);
  }

  PropertiesService.getUserProperties().setProperty(
    ROS_CTRL.courseProperty,
    String(curso.id)
  );
  escribirConfiguracion_(panel, "COURSE_ID", String(curso.id));
  return curso;
}

function crearOReutilizarTemas_(courseId, actividades) {
  var existentes = listarTemas_(courseId);
  var porNombre = {};
  existentes.forEach(function(tema) {
    porNombre[normalizar_(tema.name)] = tema;
  });

  var creados = 0;
  actividades.forEach(function(actividad) {
    if (!actividad.activa || !actividad.topico) {
      return;
    }
    var clave = normalizar_(actividad.topico);
    if (!porNombre[clave]) {
      porNombre[clave] = Classroom.Courses.Topics.create(
        { name: actividad.topico },
        courseId
      );
      creados += 1;
    }
  });

  return {
    porNombre: porNombre,
    resumen: {
      existentes: existentes.length,
      creados: creados,
      total: Object.keys(porNombre).length
    }
  };
}

function enviarInvitaciones_(panel, courseId, estudiantes) {
  var inscritos = obtenerMapaInscritos_(courseId);
  var hoja = panel.getSheetByName(ROS_CTRL.sheets.students);
  var resumen = {
    inscritos: 0,
    pendientes: 0,
    invitacionesCreadas: 0,
    errores: 0
  };

  estudiantes.forEach(function(estudiante) {
    var email = estudiante.correo.toLowerCase();
    var estado;
    var observacion = "";

    try {
      if (inscritos[email]) {
        estado = "INSCRITO";
        resumen.inscritos += 1;
      } else if (tieneInvitacionPendiente_(courseId, email)) {
        estado = "INVITACION_PENDIENTE";
        resumen.pendientes += 1;
      } else {
        Classroom.Invitations.create({
          courseId: courseId,
          userId: email,
          role: "STUDENT"
        });
        estado = "INVITADO";
        resumen.invitacionesCreadas += 1;
      }
    } catch (error) {
      estado = "ERROR";
      observacion = error && error.message ? error.message : String(error);
      resumen.errores += 1;
    }

    hoja.getRange(estudiante.row, 5, 1, 2).setValues([[estado, observacion]]);
  });

  return resumen;
}

function crearBorradores_(panel, courseId, actividades, temas) {
  var existentes = listarCourseWork_(courseId);
  var porTitulo = {};
  existentes.forEach(function(item) {
    porTitulo[normalizar_(item.title)] = item;
  });

  var hoja = panel.getSheetByName(ROS_CTRL.sheets.activities);
  var resumen = { creados: 0, reutilizados: 0, omitidos: 0, errores: 0 };

  actividades.forEach(function(actividad) {
    if (!actividad.activa) {
      resumen.omitidos += 1;
      return;
    }

    try {
      var existente = null;
      if (actividad.idClassroom) {
        try {
          existente = Classroom.Courses.CourseWork.get(
            courseId,
            actividad.idClassroom
          );
        } catch (ignorado) {
          existente = null;
        }
      }
      if (!existente) {
        existente = porTitulo[normalizar_(actividad.titulo)] || null;
      }

      if (existente) {
        hoja.getRange(actividad.row, 10, 1, 2).setValues([[
          String(existente.id),
          "REUTILIZADO"
        ]]);
        resumen.reutilizados += 1;
        return;
      }

      var body = {
        title: actividad.titulo,
        description: actividad.descripcion,
        workType: "ASSIGNMENT",
        state: "DRAFT"
      };

      configurarDestinatarios_(body, panel, courseId, actividad);

      if (actividad.puntos !== "" && !isNaN(Number(actividad.puntos))) {
        body.maxPoints = Number(actividad.puntos);
      }
      if (actividad.topico) {
        var tema = temas.porNombre[normalizar_(actividad.topico)];
        if (tema) {
          body.topicId = tema.topicId;
        }
      }
      if (actividad.materialUrl) {
        body.materials = [{ link: { url: actividad.materialUrl } }];
      }
      agregarFechaLimite_(body, actividad.fecha, actividad.hora);

      var creado = Classroom.Courses.CourseWork.create(body, courseId);
      porTitulo[normalizar_(creado.title)] = creado;
      hoja.getRange(actividad.row, 10, 1, 2).setValues([[
        String(creado.id),
        "BORRADOR_CREADO"
      ]]);
      resumen.creados += 1;
    } catch (error) {
      hoja.getRange(actividad.row, 11).setValue(
        "ERROR: " + (error && error.message ? error.message : String(error))
      );
      resumen.errores += 1;
    }
  });

  return resumen;
}

function localizarCurso_(config) {
  var ids = [];
  if (config.COURSE_ID) {
    ids.push(config.COURSE_ID);
  }
  var propertyId = PropertiesService.getUserProperties().getProperty(
    ROS_CTRL.courseProperty
  );
  if (propertyId && ids.indexOf(propertyId) === -1) {
    ids.push(propertyId);
  }

  for (var i = 0; i < ids.length; i += 1) {
    try {
      var porId = Classroom.Courses.get(ids[i]);
      if (
        porId.name === config.NOMBRE_CURSO &&
        (porId.section || "") === config.SECCION
      ) {
        return porId;
      }
    } catch (ignorado) {
      // Continúa con la búsqueda por nombre y sección.
    }
  }

  var pageToken;
  do {
    var options = { pageSize: 100 };
    if (pageToken) {
      options.pageToken = pageToken;
    }
    var response = Classroom.Courses.list(options);
    var courses = response.courses || [];
    for (var j = 0; j < courses.length; j += 1) {
      if (
        courses[j].name === config.NOMBRE_CURSO &&
        (courses[j].section || "") === config.SECCION
      ) {
        return courses[j];
      }
    }
    pageToken = response.nextPageToken;
  } while (pageToken);

  return null;
}

function listarTemas_(courseId) {
  var items = [];
  var pageToken;
  do {
    var options = { pageSize: 100 };
    if (pageToken) {
      options.pageToken = pageToken;
    }
    var response = Classroom.Courses.Topics.list(courseId, options);
    items = items.concat(response.topic || []);
    pageToken = response.nextPageToken;
  } while (pageToken);
  return items;
}

function listarCourseWork_(courseId) {
  var items = [];
  var pageToken;
  do {
    var options = {
      pageSize: 100,
      courseWorkStates: ["PUBLISHED", "DRAFT"]
    };
    if (pageToken) {
      options.pageToken = pageToken;
    }
    var response = Classroom.Courses.CourseWork.list(courseId, options);
    items = items.concat(response.courseWork || []);
    pageToken = response.nextPageToken;
  } while (pageToken);
  return items;
}

function listarEntregas_(courseId, courseWorkId) {
  var items = [];
  var pageToken;
  do {
    var options = { pageSize: 100 };
    if (pageToken) {
      options.pageToken = pageToken;
    }
    var response = Classroom.Courses.CourseWork.StudentSubmissions.list(
      courseId,
      courseWorkId,
      options
    );
    items = items.concat(response.studentSubmissions || []);
    pageToken = response.nextPageToken;
  } while (pageToken);
  return items;
}

function obtenerMapaInscritos_(courseId) {
  var result = {};
  var pageToken;
  do {
    var options = { pageSize: 100 };
    if (pageToken) {
      options.pageToken = pageToken;
    }
    var response = Classroom.Courses.Students.list(courseId, options);
    (response.students || []).forEach(function(student) {
      var email = student.profile && student.profile.emailAddress;
      if (email) {
        result[email.toLowerCase()] = true;
      }
    });
    pageToken = response.nextPageToken;
  } while (pageToken);
  return result;
}

function obtenerMapaInvitaciones_(courseId) {
  var result = {};
  var pageToken;
  do {
    var options = { courseId: courseId, pageSize: 100 };
    if (pageToken) {
      options.pageToken = pageToken;
    }
    var response = Classroom.Invitations.list(options);
    (response.invitations || []).forEach(function(invitation) {
      if (invitation.userId) {
        result[String(invitation.userId).toLowerCase()] = true;
      }
    });
    pageToken = response.nextPageToken;
  } while (pageToken);
  return result;
}

function tieneInvitacionPendiente_(courseId, email) {
  var response = Classroom.Invitations.list({
    courseId: courseId,
    userId: email,
    pageSize: 1
  });
  return Boolean(response.invitations && response.invitations.length);
}

function obtenerMapaPerfiles_(courseId) {
  var result = {};
  var pageToken;
  do {
    var options = { pageSize: 100 };
    if (pageToken) {
      options.pageToken = pageToken;
    }
    var response = Classroom.Courses.Students.list(courseId, options);
    (response.students || []).forEach(function(student) {
      var profile = student.profile || {};
      var name = profile.name || {};
      result[student.userId] = {
        email: profile.emailAddress || "",
        nombre: name.fullName || ""
      };
    });
    pageToken = response.nextPageToken;
  } while (pageToken);
  return result;
}

function auditarEstado_(courseId) {
  var course = Classroom.Courses.get(courseId);
  var students = obtenerMapaInscritos_(courseId);
  var invitations = obtenerMapaInvitaciones_(courseId);
  var topics = listarTemas_(courseId);
  var work = listarCourseWork_(courseId);
  return {
    curso: resumenCurso_(course),
    estudiantesInscritos: Object.keys(students).length,
    invitacionesPendientes: Object.keys(invitations).length,
    temas: topics.length,
    actividadesTotales: work.length,
    actividadesBorrador: work.filter(function(item) {
      return item.state === "DRAFT";
    }).length,
    actividadesPublicadas: work.filter(function(item) {
      return item.state === "PUBLISHED";
    }).length
  };
}

function leerConfiguracion_(panel) {
  var sheet = panel.getSheetByName(ROS_CTRL.sheets.config);
  var values = sheet.getDataRange().getDisplayValues();
  var config = {};
  for (var i = 1; i < values.length; i += 1) {
    var key = String(values[i][0] || "").trim();
    if (key) {
      config[key] = String(values[i][1] || "").trim();
    }
  }
  return config;
}

function leerEstudiantes_(panel, config) {
  var sheet = panel.getSheetByName(ROS_CTRL.sheets.students);
  var values = sheet.getDataRange().getDisplayValues();
  var result = [];
  for (var i = 1; i < values.length; i += 1) {
    var code = String(values[i][0] || "").trim();
    var email = String(values[i][3] || "").trim().toLowerCase();
    if (!code && !email) {
      continue;
    }
    result.push({
      row: i + 1,
      codigo: code,
      apellidos: String(values[i][1] || "").trim(),
      nombres: String(values[i][2] || "").trim(),
      correo: email,
      dominio: config.DOMINIO_ESTUDIANTES || ROS_CTRL.studentDomain
    });
  }
  return result;
}

function leerActividades_(panel) {
  var sheet = panel.getSheetByName(ROS_CTRL.sheets.activities);
  var values = sheet.getDataRange().getDisplayValues();
  var result = [];
  for (var i = 1; i < values.length; i += 1) {
    if (!String(values[i][1] || "").trim()) {
      continue;
    }
    result.push({
      row: i + 1,
      activa: normalizar_(values[i][0]) === "si",
      codigo: String(values[i][1] || "").trim(),
      titulo: String(values[i][2] || "").trim(),
      topico: String(values[i][3] || "").trim(),
      puntos: String(values[i][4] || "").trim(),
      fecha: String(values[i][5] || "").trim(),
      hora: String(values[i][6] || "").trim(),
      descripcion: String(values[i][7] || "").trim(),
      materialUrl: String(values[i][8] || "").trim(),
      idClassroom: String(values[i][9] || "").trim(),
      resultado: String(values[i][10] || "").trim(),
      asignacion: String(values[i][11] || "TODOS").trim().toUpperCase(),
      equiposObjetivo: String(values[i][12] || "").trim()
    });
  }
  return result;
}

function validarEstudiantes_(students, config) {
  var incidencias = [];
  var codes = {};
  var emails = {};
  var domain = (config.DOMINIO_ESTUDIANTES || ROS_CTRL.studentDomain).toLowerCase();

  students.forEach(function(student) {
    if (!student.codigo) {
      incidencias.push("Fila " + student.row + ": código faltante");
    } else if (codes[student.codigo]) {
      incidencias.push("Código duplicado en fila " + student.row);
    }
    codes[student.codigo] = true;

    if (!student.correo || student.correo.indexOf("@") === -1) {
      incidencias.push("Fila " + student.row + ": correo inválido");
    } else if (emails[student.correo]) {
      incidencias.push("Correo duplicado en fila " + student.row);
    } else if (student.correo.split("@").pop() !== domain) {
      incidencias.push("Fila " + student.row + ": correo fuera del dominio permitido");
    }
    emails[student.correo] = true;
  });

  if (!students.length) {
    incidencias.push("No hay estudiantes cargados en la hoja Estudiantes");
  }

  return {
    ok: incidencias.length === 0,
    total: students.length,
    codigosUnicos: Object.keys(codes).length,
    correosUnicos: Object.keys(emails).length,
    incidencias: incidencias
  };
}

function validarActividades_(activities) {
  var incidencias = [];
  var codes = {};
  var activeCount = 0;
  activities.forEach(function(activity) {
    if (codes[activity.codigo]) {
      incidencias.push("Código de actividad duplicado: " + activity.codigo);
    }
    codes[activity.codigo] = true;
    if (!activity.activa) {
      return;
    }
    activeCount += 1;
    if (!activity.titulo) {
      incidencias.push(activity.codigo + ": título faltante");
    }
    if (!activity.topico) {
      incidencias.push(activity.codigo + ": tópico faltante");
    }
    if (["TODOS", "ENTREGANTES"].indexOf(activity.asignacion) === -1) {
      incidencias.push(
        activity.codigo + ": ASIGNACION debe ser TODOS o ENTREGANTES"
      );
    }
    if (activity.fecha && !/^\d{4}-\d{2}-\d{2}$/.test(activity.fecha)) {
      incidencias.push(activity.codigo + ": use fecha AAAA-MM-DD");
    }
    if (activity.hora && !/^\d{2}:\d{2}$/.test(activity.hora)) {
      incidencias.push(activity.codigo + ": use hora HH:MM");
    }
    if (activity.hora && !activity.fecha) {
      incidencias.push(activity.codigo + ": una hora requiere fecha");
    }
  });
  return {
    ok: incidencias.length === 0,
    total: activities.length,
    activas: activeCount,
    incidencias: incidencias
  };
}

function inicializarHojaEquipos_(sheet) {
  var headers = [[
    "ACTIVO",
    "EQUIPO_ID",
    "CODIGO",
    "APELLIDOS",
    "NOMBRES",
    "CORREO",
    "GITHUB_USER",
    "ROL_INICIAL",
    "ENTREGANTE",
    "REPOSITORIO",
    "GITHUB_TEAM",
    "ESTADO_GITHUB",
    "ESTADO_CLASSROOM",
    "OBSERVACIONES"
  ]];
  if (sheet.getLastRow() === 0) {
    sheet.getRange(1, 1, 1, headers[0].length).setValues(headers);
  }
  sheet.getRange("A:N").setNumberFormat("@");
  sheet.getRange("A:N").setWrap(true);
  formatearHoja_(sheet, headers[0].length);

  var equipos = [];
  for (var i = 1; i <= 10; i += 1) {
    equipos.push("equipo-" + (i < 10 ? "0" + i : i));
  }
  aplicarValidacionLista_(sheet.getRange("A2:A"), ["SI", "NO"]);
  aplicarValidacionLista_(sheet.getRange("B2:B"), equipos);
  aplicarValidacionLista_(sheet.getRange("H2:H"), [
    "DESARROLLO",
    "INTEGRACION",
    "PRUEBAS_DOCUMENTACION"
  ]);
  aplicarValidacionLista_(sheet.getRange("I2:I"), ["SI", "NO"]);
}

function asegurarColumnasActividades_(sheet) {
  var headers = sheet.getRange(1, 1, 1, Math.max(sheet.getLastColumn(), 13))
    .getDisplayValues()[0];
  if (!headers[11]) {
    sheet.getRange(1, 12).setValue("ASIGNACION");
  }
  if (!headers[12]) {
    sheet.getRange(1, 13).setValue("EQUIPOS_OBJETIVO");
  }
  var lastRow = sheet.getLastRow();
  if (lastRow > 1) {
    var assignmentRange = sheet.getRange(2, 12, lastRow - 1, 1);
    var values = assignmentRange.getDisplayValues();
    var codes = sheet.getRange(2, 2, lastRow - 1, 1).getDisplayValues();
    for (var i = 0; i < values.length; i += 1) {
      if (!values[i][0]) {
        values[i][0] = String(codes[i][0] || "").trim() === "E01"
          ? "ENTREGANTES"
          : "TODOS";
      }
    }
    assignmentRange.setValues(values);
    aplicarValidacionLista_(assignmentRange, ["TODOS", "ENTREGANTES"]);
  }
  formatearHoja_(sheet, 13);
}

function aplicarValidacionLista_(range, values) {
  range.setDataValidation(
    SpreadsheetApp.newDataValidation()
      .requireValueInList(values, true)
      .setAllowInvalid(false)
      .build()
  );
}

function sincronizarEstudiantesEnEquipos_(panel, estudiantes) {
  var sheet = panel.getSheetByName(ROS_CTRL.sheets.teams);
  inicializarHojaEquipos_(sheet);
  var values = sheet.getDataRange().getDisplayValues();
  var porCodigo = {};
  for (var i = 1; i < values.length; i += 1) {
    var codigo = String(values[i][2] || "").trim();
    if (codigo) {
      porCodigo[codigo] = i + 1;
    }
  }
  var agregados = 0;
  var actualizados = 0;
  estudiantes.forEach(function(estudiante) {
    var row = porCodigo[estudiante.codigo];
    if (!row) {
      sheet.appendRow([
        "SI",
        "",
        estudiante.codigo,
        estudiante.apellidos,
        estudiante.nombres,
        estudiante.correo,
        "",
        "",
        "NO",
        "",
        "",
        "PENDIENTE_DATOS",
        "PENDIENTE_ASIGNACION",
        ""
      ]);
      var newRow = sheet.getLastRow();
      sheet.getRange(newRow, 10).setFormula(
        '=IF(B' + newRow + '="","","burger-kinova-"&B' + newRow + ')'
      );
      sheet.getRange(newRow, 11).setFormula(
        '=IF(B' + newRow + '="","",B' + newRow + ')'
      );
      agregados += 1;
    } else {
      sheet.getRange(row, 3, 1, 4).setValues([[
        estudiante.codigo,
        estudiante.apellidos,
        estudiante.nombres,
        estudiante.correo
      ]]);
      actualizados += 1;
    }
  });
  return {
    ok: true,
    estudiantes: estudiantes.length,
    agregados: agregados,
    actualizados: actualizados
  };
}

function leerEquipos_(panel) {
  var sheet = panel.getSheetByName(ROS_CTRL.sheets.teams);
  if (!sheet) {
    return [];
  }
  var values = sheet.getDataRange().getDisplayValues();
  var result = [];
  for (var i = 1; i < values.length; i += 1) {
    var codigo = String(values[i][2] || "").trim();
    if (!codigo) {
      continue;
    }
    result.push({
      row: i + 1,
      activo: normalizar_(values[i][0] || "SI") === "si",
      equipoId: String(values[i][1] || "").trim().toLowerCase(),
      codigo: codigo,
      apellidos: String(values[i][3] || "").trim(),
      nombres: String(values[i][4] || "").trim(),
      correo: String(values[i][5] || "").trim().toLowerCase(),
      githubUser: String(values[i][6] || "").trim(),
      rolInicial: String(values[i][7] || "").trim(),
      entregante: normalizar_(values[i][8]) === "si",
      repositorio: String(values[i][9] || "").trim(),
      githubTeam: String(values[i][10] || "").trim(),
      estadoGithub: String(values[i][11] || "").trim(),
      estadoClassroom: String(values[i][12] || "").trim()
    });
  }
  return result;
}

function validarEquiposClassroom_(equipos, estudiantes) {
  var incidencias = [];
  var activos = equipos.filter(function(item) { return item.activo; });
  var codigosEstudiantes = {};
  estudiantes.forEach(function(item) { codigosEstudiantes[item.codigo] = true; });
  var porEquipo = {};
  var codigos = {};

  activos.forEach(function(item) {
    if (!codigosEstudiantes[item.codigo]) {
      incidencias.push("Fila " + item.row + ": código no existe en Estudiantes");
    }
    if (codigos[item.codigo]) {
      incidencias.push("Código duplicado en Equipos: " + item.codigo);
    }
    codigos[item.codigo] = true;
    if (!/^equipo-(0[1-9]|10)$/.test(item.equipoId)) {
      incidencias.push("Fila " + item.row + ": EQUIPO_ID faltante o inválido");
      return;
    }
    porEquipo[item.equipoId] = porEquipo[item.equipoId] || [];
    porEquipo[item.equipoId].push(item);
  });

  estudiantes.forEach(function(item) {
    if (!codigos[item.codigo]) {
      incidencias.push("Estudiante sin fila activa en Equipos: " + item.codigo);
    }
  });

  Object.keys(porEquipo).forEach(function(equipoId) {
    var miembros = porEquipo[equipoId];
    var entregantes = miembros.filter(function(item) { return item.entregante; });
    if (miembros.length < 2 || miembros.length > 3) {
      incidencias.push(equipoId + ": debe tener 2 o 3 integrantes");
    }
    if (entregantes.length !== 1) {
      incidencias.push(equipoId + ": debe tener exactamente un ENTREGANTE=SI");
    }
  });

  return {
    ok: incidencias.length === 0,
    estudiantesActivos: activos.length,
    equiposConfigurados: Object.keys(porEquipo).length,
    incidencias: incidencias
  };
}

function validarEquiposGithub_(equipos, estudiantes) {
  var base = validarEquiposClassroom_(equipos, estudiantes);
  var incidencias = base.incidencias.slice();
  var users = {};
  equipos.filter(function(item) { return item.activo; }).forEach(function(item) {
    if (!/^[A-Za-z0-9](?:[A-Za-z0-9-]{0,37}[A-Za-z0-9])?$/.test(item.githubUser)) {
      incidencias.push("Fila " + item.row + ": GITHUB_USER faltante o inválido");
    } else if (users[item.githubUser.toLowerCase()]) {
      incidencias.push("GITHUB_USER duplicado: " + item.githubUser);
    }
    users[item.githubUser.toLowerCase()] = true;
    if (item.equipoId) {
      var repoEsperado = "burger-kinova-" + item.equipoId;
      if (item.repositorio && item.repositorio !== repoEsperado) {
        incidencias.push("Fila " + item.row + ": repositorio no corresponde al equipo");
      }
      if (item.githubTeam && item.githubTeam !== item.equipoId) {
        incidencias.push("Fila " + item.row + ": GITHUB_TEAM no corresponde al equipo");
      }
    }
  });
  return {
    ok: incidencias.length === 0,
    estudiantesActivos: base.estudiantesActivos,
    equiposConfigurados: base.equiposConfigurados,
    usuariosGithub: Object.keys(users).length,
    incidencias: incidencias
  };
}

function configurarDestinatarios_(body, panel, courseId, actividad) {
  if (actividad.asignacion !== "ENTREGANTES") {
    body.assigneeMode = "ALL_STUDENTS";
    return;
  }
  var objetivos = actividad.equiposObjetivo
    ? actividad.equiposObjetivo.split(",").map(function(value) {
        return value.trim().toLowerCase();
      })
    : [];
  var equipos = leerEquipos_(panel).filter(function(item) {
    return item.activo &&
      item.entregante &&
      (!objetivos.length || objetivos.indexOf(item.equipoId) !== -1);
  });
  var idsPorCorreo = obtenerMapaIdsPorCorreo_(courseId);
  var studentIds = [];
  equipos.forEach(function(item) {
    var id = idsPorCorreo[item.correo];
    if (!id) {
      throw new Error(
        "El entregante " + item.correo + " debe aceptar primero la invitación a Classroom."
      );
    }
    if (studentIds.indexOf(id) === -1) {
      studentIds.push(id);
    }
  });
  if (!studentIds.length) {
    throw new Error("No hay entregantes válidos para " + actividad.codigo);
  }
  body.assigneeMode = "INDIVIDUAL_STUDENTS";
  body.individualStudentsOptions = { studentIds: studentIds };
}

function obtenerMapaIdsPorCorreo_(courseId) {
  var result = {};
  var pageToken;
  do {
    var options = { pageSize: 100 };
    if (pageToken) {
      options.pageToken = pageToken;
    }
    var response = Classroom.Courses.Students.list(courseId, options);
    (response.students || []).forEach(function(student) {
      var email = student.profile && student.profile.emailAddress;
      if (email) {
        result[email.toLowerCase()] = String(student.userId);
      }
    });
    pageToken = response.nextPageToken;
  } while (pageToken);
  return result;
}

function agregarFechaLimite_(body, dateText, timeText) {
  if (!dateText) {
    return;
  }
  var time = timeText || "23:59";
  var local = new Date(dateText + "T" + time + ":00-05:00");
  if (isNaN(local.getTime())) {
    throw new Error("Fecha u hora inválida: " + dateText + " " + time);
  }
  body.dueDate = {
    year: local.getUTCFullYear(),
    month: local.getUTCMonth() + 1,
    day: local.getUTCDate()
  };
  body.dueTime = {
    hours: local.getUTCHours(),
    minutes: local.getUTCMinutes()
  };
}

function extraerAdjuntos_(submission) {
  var assignment = submission.assignmentSubmission || {};
  var attachments = assignment.attachments || [];
  var titles = [];
  var links = [];
  attachments.forEach(function(attachment) {
    if (attachment.driveFile) {
      var file = attachment.driveFile;
      titles.push(file.title || "Archivo de Drive");
      links.push(file.alternateLink || file.id || "");
    } else if (attachment.link) {
      titles.push(attachment.link.title || "Enlace");
      links.push(attachment.link.url || "");
    } else if (attachment.youTubeVideo) {
      titles.push(attachment.youTubeVideo.title || "YouTube");
      links.push(attachment.youTubeVideo.alternateLink || "");
    }
  });
  return { titulos: titles, enlaces: links };
}

function obtenerPanelSiExiste_() {
  var id = PropertiesService.getUserProperties().getProperty(
    ROS_CTRL.panelProperty
  );
  if (!id) {
    return null;
  }
  try {
    return SpreadsheetApp.openById(id);
  } catch (ignorado) {
    return null;
  }
}

function obtenerPanel_() {
  var panel = obtenerPanelSiExiste_();
  if (!panel) {
    throw new Error(
      "No existe el panel privado. Ejecute instalarControladorROS() una vez."
    );
  }
  return panel;
}

function obtenerCourseId_(config) {
  var id = config.COURSE_ID || PropertiesService.getUserProperties().getProperty(
    ROS_CTRL.courseProperty
  );
  if (!id) {
    throw new Error("No hay COURSE_ID. Ejecute primero configurarTodoROS().");
  }
  return String(id);
}

function asegurarHoja_(panel, name) {
  return panel.getSheetByName(name) || panel.insertSheet(name);
}

function escribirConfiguracion_(panel, key, value) {
  var sheet = panel.getSheetByName(ROS_CTRL.sheets.config);
  var values = sheet.getDataRange().getDisplayValues();
  for (var i = 1; i < values.length; i += 1) {
    if (String(values[i][0] || "").trim() === key) {
      sheet.getRange(i + 1, 2).setValue(value);
      return;
    }
  }
  sheet.appendRow([key, value, "Agregado automáticamente por el controlador."]);
}

function exigirModoAplicar_(config) {
  if (normalizar_(config.MODO) !== "aplicar") {
    throw new Error(
      "Operación cancelada: escriba APLICAR en Configuracion > MODO después de revisar la vista previa."
    );
  }
}

function registrarAuditoria_(panel, action, result) {
  try {
    var sheet = panel.getSheetByName(ROS_CTRL.sheets.audit);
    if (!sheet) {
      return;
    }
    sheet.appendRow([
      new Date(),
      action,
      result && result.ok === true ? "SI" : "NO",
      JSON.stringify(result)
    ]);
  } catch (ignorado) {
    // La auditoría nunca debe ocultar el resultado de la operación principal.
  }
}

function formatearHoja_(sheet, columns) {
  sheet.setFrozenRows(1);
  sheet.getRange(1, 1, 1, columns)
    .setFontWeight("bold")
    .setBackground("#17365D")
    .setFontColor("#FFFFFF");
  sheet.autoResizeColumns(1, columns);
}

function resumenCurso_(course) {
  return {
    id: course.id,
    nombre: course.name,
    seccion: course.section || "",
    estado: course.courseState,
    enlace: course.alternateLink || ""
  };
}

function resultadoError_(message, error) {
  return {
    ok: false,
    mensaje: message,
    detalle: error && error.message ? error.message : String(error)
  };
}

function normalizar_(value) {
  return String(value || "")
    .toLowerCase()
    .normalize("NFD")
    .replace(/[\u0300-\u036f]/g, "")
    .trim();
}

function valorSeguro_(value) {
  return value === undefined || value === null ? "" : value;
}
