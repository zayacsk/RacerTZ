using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform target; // Игрок, за которой следует камера
    public Vector3 offset = new Vector3(0, 5, -10); // Смещение камеры относительно цели
    public float followSpeed = 10f; // Скорость перемещения камеры
    public float rotationSpeed = 10f; // Скорость поворота камеры

    private void Awake()
    {
        Application.targetFrameRate = 60;
    }

    private void Update()
    {
        if (target == null)
        {
            Debug.LogWarning("Target не установлен для CameraFollow!");
            return;
        }

        // Следование камеры за машиной
        Vector3 desiredPosition = target.position + target.TransformDirection(offset);
        transform.position = Vector3.Lerp(transform.position, desiredPosition, Time.deltaTime * followSpeed);

        // Поворот камеры, чтобы смотреть на машину
        Quaternion desiredRotation = Quaternion.LookRotation(target.position - transform.position);
        transform.rotation = Quaternion.Slerp(transform.rotation, desiredRotation, Time.deltaTime * rotationSpeed);
    }
}