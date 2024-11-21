using System.Collections;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.UI;

public class SceneController : MonoBehaviour
{
    public GameObject player;
    public GameObject ghostPrefab;
    public Text typeTxt;
    public AudioListener listener;
    public GameObject preStartTxt;

    private MovementRecorder _movementRecorder;
    private GameObject _ghost;
    private Player _player;

    private void Start()
    {
        if (player != null)
        {
            _movementRecorder = player.GetComponent<MovementRecorder>();
            _player = player.GetComponent<Player>();
            Time.timeScale = 0f;
        }
    }

    private void Update()
    {
        if (Input.GetKeyDown(KeyCode.E))
        {
            StartGame();
        }
        if (Input.GetKeyDown(KeyCode.R))
        {
            Restart();
        }
        if (Input.GetKeyDown(KeyCode.G))
        {
            _movementRecorder.ResetData();
        }
    }

    public void StartGame()
    {
        if (_movementRecorder != null)
        {
            if (_movementRecorder.previousSessionTransforms.Count > 0)
            {
                _ghost = Instantiate(ghostPrefab, _movementRecorder.previousSessionTransforms[0].position, _movementRecorder.previousSessionTransforms[0].rotation);
                StartCoroutine(MoveGhost());
                typeTxt.text = "With ghost";
            }
            else
            {
                typeTxt.text = "Without ghost";
            }
            _movementRecorder.StartRecording();
        }
        preStartTxt.SetActive(false);
        listener.enabled = true;
        Time.timeScale = 1f;
    }

    private IEnumerator MoveGhost()
    {
        if (_ghost == null || _movementRecorder.previousSessionTransforms.Count == 0)
            yield break;

        foreach (var transformData in _movementRecorder.previousSessionTransforms)
        {
            Vector3 newPosition = transformData.position;
            newPosition.y -= 0.8f;

            _ghost.transform.position = newPosition;
            _ghost.transform.rotation = transformData.rotation;

            yield return null;
        }
    }

    public void Restart()
    {
        if (_player.HasReachedFinish())
        {
            if (_movementRecorder != null)
            {
                _movementRecorder.SaveCurrentSession();
            }
        }

        SceneManager.LoadScene(0);
    }
}