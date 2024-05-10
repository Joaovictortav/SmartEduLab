using Microsoft.AspNetCore.Builder;
using Microsoft.AspNetCore.Hosting;
using Microsoft.Extensions.DependencyInjection;
using Microsoft.Extensions.Hosting;
using MQTTnet.AspNetCore;

namespace BackFanPlate
{
    public class Server
    {
        public static async Task Run()
        {
            var host = Host.CreateDefaultBuilder(Array.Empty<string>())
                .ConfigureWebHostDefaults(webBuilder =>
                {
                    webBuilder.UseKestrel(o => { o.ListenAnyIP(1883); });
                    webBuilder.UseKestrel(o => { o.ListenAnyIP(1884, l => l.UseMqtt()); });
                    webBuilder.UseStartup<Startup>();
                });

            await host.RunConsoleAsync();
        }
    }

    public sealed class Startup
    {
        public void Configure(IApplicationBuilder app, IWebHostEnvironment environment, MqttController mqttController)
        {
            app.UseRouting();

            app.UseEndpoints(endpoints =>
            {
                endpoints.MapConnectionHandler<MqttConnectionHandler>(
                    "/mqtt",
                    httpConnectionDispatcherOptions => httpConnectionDispatcherOptions.WebSockets.SubProtocolSelector =
                        protocolList => protocolList.FirstOrDefault() ?? string.Empty);
            });

            app.UseMqttServer(server =>
            {
                server.ValidatingConnectionAsync += MqttController.ValidatingConnectionAsync;
                server.ClientDisconnectedAsync += MqttController.ClientDisconnectedAsync;
                server.InterceptingPublishAsync += MqttController.InterceptingPublishAsync;
                server.InterceptingSubscriptionAsync += MqttController.InterceptingSubscriptionAsync;
                server.InterceptingUnsubscriptionAsync += MqttController.InterceptingUnsubscriptionAsync;
            });
        }

        public void ConfigureServices(IServiceCollection services)
        {
            services.AddHostedMqttServer(optionsBuilder =>
            {
                optionsBuilder.WithDefaultEndpoint();
            });

            services.AddMqttConnectionHandler();
            services.AddConnections();

            services.AddSingleton<MqttController>();
        }
    }
}
