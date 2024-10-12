using UnityEngine.Assertions;
using Unity.Collections;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using UnityEditor;
using System.Linq;

// Naive priority queue implementation for small scale problems
// public class PriorityQueue<T> 
// {
//     // I'm using an unsorted array for this example, but ideally this
//     // would be a binary heap. There's an open issue for adding a binary
//     // heap to the standard C# library.
//     private readonly List<Tuple<T, float>> elements = new List<Tuple<T, float>>();

//     public int Count => elements.Count;

//     public void Enqueue(T item, float priority)
//     {
//         elements.Add(Tuple.Create(item, priority));
//     }

//     public T Dequeue()
//     {
//         // Returns the lowest priority element first
//         int bestIndex = 0;

//         for (int i = 0; i < elements.Count; i++)
//         {
//             if (elements[i].Item2 < elements[bestIndex].Item2)
//             {
//                 bestIndex = i;
//             }
//         }

//         T bestItem = elements[bestIndex].Item1;
//         elements.RemoveAt(bestIndex);
//         return bestItem;
//     }
// }

// Or:
public class PriorityQueue<TElement, TPriority>
{
    private List<Tuple<TElement, TPriority>> elements = new List<Tuple<TElement, TPriority>>();

    public int Count
    {
        get { return elements.Count; }
    }

    public void Clear()
    {
        elements.Clear();
    }

    public void Enqueue(TElement item, TPriority priority)
    {
        elements.Add(Tuple.Create(item, priority));
    }

    public TElement Dequeue()
    {
        Comparer<TPriority> comparer = Comparer<TPriority>.Default;
        int bestIndex = 0;

        for (int i = 0; i < elements.Count; i++)
        {
            if (comparer.Compare(elements[i].Item2, elements[bestIndex].Item2) < 0)
            {
                bestIndex = i;
            }
        }

        TElement bestItem = elements[bestIndex].Item1;
        elements.RemoveAt(bestIndex);
        return bestItem;
    }
}