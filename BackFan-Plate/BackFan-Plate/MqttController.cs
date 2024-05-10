using System.Text;
using MQTTnet.Protocol;
using MQTTnet.Server;
using Newtonsoft.Json;
using Util;

namespace BackFanPlate;

public class MqttController
{
    public MqttController() { }

    public static Task ClientDisconnectedAsync(ClientDisconnectedEventArgs arg)
    {
        Console.WriteLine($"SERVER MQTT: Client disconnected '{arg.ClientId}'");
        return Task.CompletedTask;
    }

    public static Task ValidatingConnectionAsync(ValidatingConnectionEventArgs arg)
    {
        Console.WriteLine($"SERVER MQTT: Client connected '{arg.ClientId}'");
        arg.ReasonCode = MqttConnectReasonCode.Success;
        return Task.CompletedTask;
    }

    public static Task InterceptingPublishAsync(InterceptingPublishEventArgs arg)
    {
        var topic = arg.ApplicationMessage.Topic;
        var device = topic.Split("/").LastOrDefault();

        var payload = arg.ApplicationMessage != null ? 
            Encoding.UTF8.GetString(arg.ApplicationMessage.Payload!) : string.Empty;

        if (!topic.StartsWith("device/", StringComparison.OrdinalIgnoreCase) || string.IsNullOrEmpty(payload))
            return Task.CompletedTask;

        var incomingValues = JsonConvert.DeserializeObject<List<List<double>>>(payload);
    
        if (incomingValues is not { Count: 3 })
            throw new InvalidOperationException("Payload deve conter 3 listas, cada uma com 100 elementos.");
        
        var redis = Caching.Get(topic);
        var cachedLists = redis is null ? [..Enumerable.Range(0, 3).Select(_ => new List<double>())] : JsonConvert.DeserializeObject<List<List<double>>>(redis);
        var newList = new List<List<double>>(Enumerable.Range(0, 3).Select(_ => new List<double>()));
    
        for (var i = 0; i < 3; i++)
        {
            if (cachedLists == null) continue;
            
            var remainingCapacity = 100 - cachedLists[i].Count;
            if (remainingCapacity >= incomingValues[i].Count)
            {
                cachedLists[i].AddRange(incomingValues[i]);
                continue;
            }
        
            cachedLists[i].AddRange(incomingValues[i].Take(remainingCapacity));
            newList[i].AddRange(incomingValues[i].Skip(remainingCapacity));
        }
        
        if (cachedLists?.FirstOrDefault()?.Count == 100)
        {
            SaveToFile(cachedLists, device);
            Caching.Remove(topic);
        }
        else
        {
            Caching.Set(topic, JsonConvert.SerializeObject(cachedLists));
        }
        
        if (newList.FirstOrDefault()?.Count > 0)
            Caching.Set(topic, JsonConvert.SerializeObject(newList));
    
        return Task.CompletedTask;
    }
    
    private static void SaveToFile(List<List<double>> data, string device)
    {
        var fileName = $"data_{device}_{DateTime.Now:yyyyMMddHHmmss}.txt";
        var filePath = Path.Combine(Directory.GetCurrentDirectory(), fileName);

        using (var writer = new StreamWriter(filePath))
        {
            foreach (var list in data)
            {
                writer.WriteLine(string.Join(",", list));
            }
        }

        Console.WriteLine($"Dados salvos em {filePath}");
    }

    public static Task InterceptingSubscriptionAsync(InterceptingSubscriptionEventArgs arg)
    {
        Console.WriteLine($"SERVER MQTT: Client '{arg.ClientId}' subscribed to topic '{arg.TopicFilter.Topic}'");
        return Task.CompletedTask;
    }

    public static Task InterceptingUnsubscriptionAsync(InterceptingUnsubscriptionEventArgs arg)
    {
        Console.WriteLine($"SERVER MQTT: Client '{arg.ClientId}' unsubscribed to topic '{arg.Topic}'");
        return Task.CompletedTask;
    }
}