using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;

namespace TheraBytes.BetterUi
{
    public interface ISizeConfigCollection
    {
        string GetCurrentConfigName();
        void MarkDirty();
        bool IsDirty { get; }
        void Sort();
    }

    [Serializable]
    public class SizeConfigCollection<T> : ISizeConfigCollection
        where T : class, IScreenConfigConnection
    {
        [SerializeField]
        List<T> items = new List<T>();
        
        public IReadOnlyList<T> Items { get { return items; } }

        bool isDirty = true;
        public bool IsDirty { get { return isDirty; } }

        public void AddItem(T item)
        {
            items.Add(item);
            MarkDirty();
        }

        public void Sort()
        {
            if (!isDirty)
                return;

            List<string> order = ResolutionMonitor.Instance.OptimizedScreens.Select(o => o.Name).ToList();
            items.Sort((a, b) => order.IndexOf(a.ScreenConfigName).CompareTo(order.IndexOf(b.ScreenConfigName)));

            isDirty = false;
        }

        public string GetCurrentConfigName()
        {
            T result = GetCurrentItem(null);

            if (result != null)
                return result.ScreenConfigName;

            return null;
        }

        public T GetItemForConfig(string configName, T fallback)
        {
            foreach(var itm in items)
            {
                if (itm.ScreenConfigName == configName)
                    return itm;
            }

            return fallback;
        }

        public T GetCurrentItem(T fallback)
        {
            // if there is no config matching the screen
            if (ResolutionMonitor.CurrentScreenConfiguration == null)
                return fallback;

            Sort();
#if UNITY_EDITOR
            
            // simulation
            var config = ResolutionMonitor.SimulatedScreenConfig;
            if (config != null)
            {
                if (Items.Any(o => o.ScreenConfigName == config.Name))
                {
                    return Items.First(o => o.ScreenConfigName == config.Name);
                }
            }
#endif

            // search for screen config
            foreach (T item in items)
            {
                if (string.IsNullOrEmpty(item.ScreenConfigName))
                    return fallback;

                var c = ResolutionMonitor.GetConfig(item.ScreenConfigName);
                if(c != null && c.IsActive)
                {
                    return item;
                }
            }
            
            // fallback logic
            foreach (var conf in ResolutionMonitor.GetCurrentScreenConfigurations())
            {
                foreach (var c in conf.Fallbacks)
                {
                    var matchingItem = items.FirstOrDefault(o => o.ScreenConfigName == c);
                    if (matchingItem != null)
                        return matchingItem;
                }
            }

            // final fallback
            return fallback;
        }

        public void MarkDirty()
        {
            isDirty = true;
        }
    }
}
