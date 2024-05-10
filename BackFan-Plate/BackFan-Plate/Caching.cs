#nullable enable
using Microsoft.Extensions.Caching.Memory;

namespace Util
{
    public class Caching
    {
        private static IMemoryCache _cache = new MemoryCache(new MemoryCacheOptions());
        private static readonly ReaderWriterLockSlim Lock = new ();

        public static bool Set(string key, string value, int minutes = 12)
        {
            try
            {
                Lock.EnterWriteLock();
                _cache.Set(key, value, TimeSpan.FromMinutes(minutes));
                return true;
            }
            catch
            {
                return false;
            }
            finally
            {
                Lock.ExitWriteLock();
            }
        }

        public static string? Get(string key)
        {
            try
            {
                Lock.EnterReadLock();
                return _cache.TryGetValue(key, out string? value) ? value : null;
            }
            finally
            {
                Lock.ExitReadLock();
            }
        }

        public static bool Remove(string key)
        {
            try
            {
                Lock.EnterWriteLock();
                _cache.Remove(key);
                return true;
            }
            catch
            {
                return false;
            }
            finally
            {
                Lock.ExitWriteLock();
            }
        }
        
        public static bool RemoveAll()
        {
            try
            {
                Lock.EnterWriteLock();
                _cache = new MemoryCache(new MemoryCacheOptions());
                return true;
            }
            catch
            {
                return false;
            }
            finally
            {
                Lock.ExitWriteLock();
            }
        }
    }
}