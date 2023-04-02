using UnityEngine;
using InteractiveSound;

namespace ParticleEffect
{
    public class SpecificSpawnPart : MonoBehaviour
    {
        /// <summary>
        /// Maximum times that particle system can cycle
        /// Negative value indicate infinite count
        /// </summary>
        public int maxAllCycleCount = -1;
        [HideInInspector]
        public ParticleSystem ps;
        float initParticleMinNum;
        float initParticleMaxNum;
        /// <summary>
        /// Always less than maxAllCycleCount
        /// </summary>
        int maxLoopCycleCount = 1;
        bool isNewCycle = true;

        InteractiveSoundBase soundBase;
        // Start is called before the first frame update
        void Awake()
        {
            ps = GetComponent<ParticleSystem>();
            soundBase = GetComponent<InteractiveSoundBase>();

            // We don't want particle system kill itself after it stops
            var psMain = ps.main;
            psMain.stopAction = ParticleSystemStopAction.None;

            var emission = ps.emission;
            if (emission.burstCount > 0)
            {
                var burst = emission.GetBurst(0);
                initParticleMinNum = burst.minCount;
                initParticleMaxNum = burst.maxCount;
            }
            else
            {
                var rate = emission.rateOverTime;
                initParticleMinNum = rate.constantMin;
                initParticleMaxNum = rate.constantMax;
            }

            StopSpawn();
        }

        void FixedUpdate() 
        {
            float deltaTime = Time.fixedDeltaTime;
            FixedUpdateSpawnState(deltaTime);    
        }

        void FixedUpdateSpawnState(float deltaTime)
        {
            if (maxAllCycleCount != 0 && !ps.isStopped)
            {
                if (ps.time + deltaTime >= ps.main.duration)
                {
                    if (maxAllCycleCount > 0)
                        maxAllCycleCount--;

                    if (maxLoopCycleCount > 0)
                        maxLoopCycleCount--;

                    if (maxLoopCycleCount == 0)
                        StopSpawn();
                    else if (!isNewCycle)
                        isNewCycle = true;
                }
            }
        }

        public void StopSpawn()
        {
            if (!ps.isStopped)
                ps.Stop(true, ParticleSystemStopBehavior.StopEmitting);
        }
        void StartSpawn(int nowMaxLoopCycleCount = -1, float rateScaler = 1f)
        {
            if (ps.isStopped)
            {
                ps.Play(true);

                if (maxAllCycleCount < 0)
                    maxLoopCycleCount = nowMaxLoopCycleCount;
                else if (nowMaxLoopCycleCount < 0)
                    maxLoopCycleCount = maxAllCycleCount;
                else
                    maxLoopCycleCount = Mathf.Min(maxAllCycleCount, nowMaxLoopCycleCount);

                isNewCycle = true;

                if (soundBase != null)
                    soundBase.PlaySoundOnce(rateScaler);
            }
        }
        void ScaleSpawnRate(float rateScaler)
        {
            if (isNewCycle)
            {
                var emission = ps.emission;
                if (emission.burstCount > 0)
                {
                    var burst = emission.GetBurst(0);
                    burst.minCount = (short)(initParticleMinNum * rateScaler);
                    burst.maxCount = (short)(initParticleMaxNum * rateScaler);
                    emission.SetBurst(0, burst);
                }
                else
                {
                    var rate = emission.rateOverTime;
                    rate.constantMin = initParticleMinNum * rateScaler;
                    rate.constantMax = initParticleMaxNum * rateScaler;
                    emission.rateOverTime = rate;
                }

                isNewCycle = false;
            }
        }
        public void ScaleAndStart(float rateScaler = 1f, int nowMaxLoopCycleCount = -1)
        {
            if (maxAllCycleCount != 0 && rateScaler > 0f)
            {
                ScaleSpawnRate(rateScaler);
                StartSpawn(nowMaxLoopCycleCount, rateScaler);
            }
            else
            {
                StopSpawn();
            }
        }
    }
}
