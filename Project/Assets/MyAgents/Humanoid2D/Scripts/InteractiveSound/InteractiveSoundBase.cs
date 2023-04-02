using UnityEngine;

namespace InteractiveSound
{
    public class InteractiveSoundBase : MonoBehaviour
    {
        [Space(5)][Header("Audio Clips Parameters")]
        public string soundTypeName;
        public SoundClip[] allSoundClips;

        [System.Serializable]
        public class SoundClip
        {
            public AudioClip sound;
            public float baseVolumeScale = 1f;
        }

        protected AudioSource audioSource;
        protected float originalPitch;
        protected int lastSoundClipIndex = -1;
        void Start()
        {
            audioSource = GetComponent<AudioSource>();

            originalPitch = audioSource.pitch;
        }

        /// <summary>
        /// AudioSource can play arbitrary number of sound clips at same time
        /// </summary>
        public virtual void PlaySoundOnce(float nowVolumeScale = 1f, float nowPitchAlpha = 0f, int soundClipIndex = -1)
        {
            audioSource.pitch = Mathf.Lerp(1f, originalPitch, nowPitchAlpha);

            if (soundClipIndex < 0 || soundClipIndex >= allSoundClips.Length)
                soundClipIndex = Random.Range(0, allSoundClips.Length);

            var soundClip = allSoundClips[soundClipIndex];
            audioSource.PlayOneShot(soundClip.sound, soundClip.baseVolumeScale * nowVolumeScale);

            lastSoundClipIndex = soundClipIndex;
        }
    }
}
