using UnityEngine;

public class Player : MonoBehaviour
{
    public GameObject finishedTxt;
    public AudioListener listener;

    private bool _reachedFinish = false;

    private void OnTriggerEnter(Collider other)
    {
        if (other.CompareTag("Finish"))
        {
            Time.timeScale = 0f;
            finishedTxt.SetActive(true);
            listener.enabled = false;

            _reachedFinish = true;
        }
    }

    public bool HasReachedFinish()
    {
        return _reachedFinish;
    }
}
