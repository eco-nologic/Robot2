Import("env")

def post_upload_action(source, target, env):
    print("\n[Post-Upload Hook] Firmware successfully flashed.")
    print("[Post-Upload Hook] Triggering automated LittleFS upload for 'data' folder...")
    
    # This command executes the 'uploadfs' target for the current active environment
    # to ensure the web dashboard files are synchronized with the code.
    env.Execute("pio run -e %s -t uploadfs" % env.get("PIOENV"))

env.AddPostAction("upload", post_upload_action)