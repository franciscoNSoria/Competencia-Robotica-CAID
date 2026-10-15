# CAD

Archivos fuente de diseño (SolidWorks de la organización y/o archivos de Fusion 360 del equipo).

- Los formatos nativos (`.SLDPRT`, `.SLDASM`, `.f3d`) son binarios y no se pueden comparar entre
  versiones: subir solo cuando haya un cambio real y explicarlo en el mensaje del commit.
- Las cotas que usa el software (separación entre ruedas, posición de sensores, cámara y brazo) se
  anotan en un archivo de texto versionable (`dimensions.yaml`, que crea la tarea de descripción
  del robot), para no depender de abrir el CAD.
