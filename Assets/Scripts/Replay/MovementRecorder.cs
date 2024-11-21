using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.SceneManagement;

public class MovementRecorder : MonoBehaviour
{
    public List<TransformData> previousSessionTransforms = new List<TransformData>();
    public List<TransformData> currentSessionTransforms = new List<TransformData>();

    private bool isRecording = false;

    private void Start()
    {
        LoadPreviousSession();
    }

    private void Update()
    {
        if (isRecording)
        {
            RecordCurrentTransform();
        }
    }

    private void RecordCurrentTransform()
    {
        TransformData data = new TransformData(transform.position, transform.rotation);
        currentSessionTransforms.Add(data);
    }

    public void StartRecording()
    {
        isRecording = true;
        currentSessionTransforms.Clear();
    }

    public void StopRecording()
    {
        isRecording = false;
    }

    public void SaveCurrentSession()
    {
        string json = JsonUtility.ToJson(new TransformDataList(currentSessionTransforms));
        PlayerPrefs.SetString("PlayerMovement", json);
        PlayerPrefs.Save();
    }

    public void LoadPreviousSession()
    {
        string json = PlayerPrefs.GetString("PlayerMovement", "");
        if (!string.IsNullOrEmpty(json))
        {
            TransformDataList data = JsonUtility.FromJson<TransformDataList>(json);
            previousSessionTransforms = data.transforms;
        }
    }

    public void ResetData()
    {
        PlayerPrefs.DeleteAll();
        Debug.Log("Reset Data");
        SceneManager.LoadScene(0);
    }

    [System.Serializable]
    public class TransformData
    {
        public Vector3 position;
        public Quaternion rotation;

        public TransformData(Vector3 position, Quaternion rotation)
        {
            this.position = position;
            this.rotation = rotation;
        }
    }

    [System.Serializable]
    public class TransformDataList
    {
        public List<TransformData> transforms;

        public TransformDataList(List<TransformData> transforms)
        {
            this.transforms = transforms;
        }
    }
}