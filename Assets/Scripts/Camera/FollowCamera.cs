using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    public Transform target; // �����, �� ������� ������� ������
    public Vector3 offset = new Vector3(0, 5, -10); // �������� ������ ������������ ����
    public float followSpeed = 10f; // �������� ����������� ������
    public float rotationSpeed = 10f; // �������� �������� ������

    private void Awake()
    {
        Application.targetFrameRate = 60;
    }

    private void Update()
    {
        if (target == null)
        {
            Debug.LogWarning("Target �� ���������� ��� CameraFollow!");
            return;
        }

        // ���������� ������ �� �������
        Vector3 desiredPosition = target.position + target.TransformDirection(offset);
        transform.position = Vector3.Lerp(transform.position, desiredPosition, Time.deltaTime * followSpeed);

        // ������� ������, ����� �������� �� ������
        Quaternion desiredRotation = Quaternion.LookRotation(target.position - transform.position);
        transform.rotation = Quaternion.Slerp(transform.rotation, desiredRotation, Time.deltaTime * rotationSpeed);
    }
}