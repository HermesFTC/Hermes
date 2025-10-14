package gay.zharel.hermes.logs

import android.annotation.SuppressLint
import android.content.Context
import android.content.res.AssetManager
import com.fasterxml.jackson.core.JsonFactory
import com.fasterxml.jackson.databind.ObjectMapper
import com.qualcomm.robotcore.util.RobotLog
import com.qualcomm.robotcore.util.WebHandlerManager
import fi.iki.elonen.NanoHTTPD
import fi.iki.elonen.NanoHTTPD.IHTTPSession
import org.firstinspires.ftc.ftccommon.external.WebHandlerRegistrar
import org.firstinspires.ftc.robotcore.internal.system.AppUtil
import org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
import org.firstinspires.ftc.robotserver.internal.webserver.MimeTypesUtil
import java.io.File
import java.io.FileInputStream
import java.io.IOException
import java.text.SimpleDateFormat
import java.util.*

val CONFIG_ROOT = File(AppUtil.ROOT_FOLDER, "Hermes/config")

@SuppressLint("SimpleDateFormat")
private val DATE_FORMAT = SimpleDateFormat("yyyy_MM_dd__HH_mm_ss_SSS")

private fun newStaticAssetHandler(assetManager: AssetManager, file: String): WebHandler {
    return WebHandler { session: IHTTPSession ->
        if (session.method == NanoHTTPD.Method.GET) {
            val mimeType = MimeTypesUtil.determineMimeType(file)
            NanoHTTPD.newChunkedResponse(NanoHTTPD.Response.Status.OK,
                    mimeType, assetManager.open(file))
        } else {
            NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.NOT_FOUND,
                    NanoHTTPD.MIME_PLAINTEXT, "")
        }
    }
}

private fun registerAssetsUnderPath(webHandlerManager: WebHandlerManager,
                                    assetManager: AssetManager, path: String) {
    try {
        val list = assetManager.list("web/$path") ?: return
        if (list.isNotEmpty()) {
            for (file in list) {
                registerAssetsUnderPath(webHandlerManager, assetManager, "$path/$file")
            }
        } else {
            webHandlerManager.register("/$path", newStaticAssetHandler(assetManager, "web/$path"))
        }
    } catch (e: IOException) {
        RobotLog.setGlobalErrorMsg(RuntimeException(e),
                "unable to register tuning web routes")
    }
}

private fun newLatestFileHandler(dir: File): WebHandler {
    return WebHandler {
        val files = dir.listFiles()
        if (files != null) {
            var mostRecentLastModified: Long = 0
            var mostRecentFile: File? = null
            for (f in files) {
                val lastModified = f.lastModified()
                if (lastModified > mostRecentLastModified) {
                    mostRecentLastModified = lastModified
                    mostRecentFile = f
                }
            }
            if (mostRecentFile != null) {
                val mimeType = MimeTypesUtil.determineMimeType(mostRecentFile.name)
                val res = NanoHTTPD.newChunkedResponse(NanoHTTPD.Response.Status.OK,
                        mimeType,
                        FileInputStream(mostRecentFile))
                res.addHeader("X-Filename", mostRecentFile.name)
                return@WebHandler res
            }
        }
        NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.NOT_FOUND,
                NanoHTTPD.MIME_PLAINTEXT, "")
    }
}

private fun newStaticFileHandler(f: File): WebHandler {
    return WebHandler { session: IHTTPSession ->
        if (session.method == NanoHTTPD.Method.GET) {
            val mimeType = MimeTypesUtil.determineMimeType(f.name)
            return@WebHandler NanoHTTPD.newChunkedResponse(NanoHTTPD.Response.Status.OK,
                    mimeType, FileInputStream(f))
        } else {
            return@WebHandler NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.NOT_FOUND,
                    NanoHTTPD.MIME_PLAINTEXT, "")
        }
    }
}

object TuningFiles {
    private val ROOT = File(AppUtil.ROOT_FOLDER.toString() + "/RoadRunner/tuning/")
    private val IO_LOCK = Any()
    private var whm // guarded by ioLock
            : WebHandlerManager? = null

    private fun getFileTypeDir(ty: FileType): File {
        return File(ROOT, ty.baseName)
    }

    fun save(ty: FileType, data: Any?) {
        synchronized(IO_LOCK) {
            val f = File(getFileTypeDir(ty), System.currentTimeMillis().toString() + ".json")
            try {
                ObjectMapper(JsonFactory())
                        .writerWithDefaultPrettyPrinter()
                        .writeValue(f, data)
                if (whm != null) {
                    whm!!.register("/tuning/" + ty.baseName + "/" + f.name,
                            newStaticFileHandler(f))
                }
            } catch (e: IOException) {
                RobotLog.setGlobalErrorMsg(RuntimeException(e),
                        "Unable to write data to " + f.absolutePath)
            }
        }
    }

    @WebHandlerRegistrar
    @JvmStatic
    fun registerRoutes(context: Context, manager: WebHandlerManager) {
        synchronized(IO_LOCK) {
            val assetManager = context.assets
            registerAssetsUnderPath(manager, assetManager, "assets")
            registerAssetsUnderPath(manager, assetManager, "tuning")
            for (ty in FileType.entries) {
                val base = "/tuning/" + ty.baseName
                val dir = getFileTypeDir(ty)
                dir.mkdirs()
                manager.register("$base/latest.json",
                        newLatestFileHandler(dir))
                for (f in Objects.requireNonNull(dir.listFiles())) {
                    manager.register(base + "/" + f.name,
                            newStaticFileHandler(f))
                }
            }
            whm = manager
        }
    }

    enum class FileType(val baseName: String) {
        FORWARD_RAMP("forward-ramp"),
        FORWARD_STEP("forward-step"),
        LATERAL_RAMP("lateral-ramp"),
        ANGULAR_RAMP("angular-ramp"),
        ANGULAR_STEP("angular-step"),
        ACCEL("accel");
    }
}

object ConfigFiles {
    @WebHandlerRegistrar
    @JvmStatic
    fun registerRoutes(context: Context, manager: WebHandlerManager) {
        manager.register("/config") {
            val sb = StringBuilder()
            sb.append("<!doctype html><html><head><title>Config</title></head><body>")

            // Added a form for uploading files
            sb.append("<h2>Upload a Config File</h2>")
            sb.append("<form action=\"/config/upload\" method=\"post\" enctype=\"multipart/form-data\">")
            sb.append("<input type=\"file\" name=\"uploadFile\" required><br><br>")
            sb.append("<input type=\"submit\" value=\"Upload\">")
            sb.append("</form>")

            sb.append("<h2>Available Config Files</h2>")
            sb.append("<ul>")
            val fs = CONFIG_ROOT.listFiles()!!
            fs.sortByDescending { it.lastModified() }
            for (f in fs) {
                sb.append("<li><a href=\"/config/download?file=")
                sb.append(f.name)
                sb.append("\" download=\"")
                sb.append(f.name)
                sb.append("\">")
                sb.append(f.name)
                sb.append("</a> (")
                sb.append(f.length())
                sb.append(" bytes)</li>")
            }
            sb.append("</ul></body></html>")
            NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK,
                NanoHTTPD.MIME_HTML, sb.toString())
        }

        manager.register("/config/download") { session: IHTTPSession ->
            val pairs = session.queryParameterString.split("&".toRegex()).dropLastWhile { it.isEmpty() }.toTypedArray()
            if (pairs.size != 1) {
                return@register NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST,
                    NanoHTTPD.MIME_PLAINTEXT, "expected one query parameter, got " + pairs.size)
            }
            val parts = pairs[0].split("=".toRegex()).dropLastWhile { it.isEmpty() }.toTypedArray()
            if (parts[0] != "file") {
                return@register NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST,
                    NanoHTTPD.MIME_PLAINTEXT, "expected file query parameter, got " + parts[0])
            }
            val f = File(CONFIG_ROOT, parts[1])
            if (!f.exists()) {
                return@register NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.NOT_FOUND,
                    NanoHTTPD.MIME_PLAINTEXT, "file $f doesn't exist")
            }
            NanoHTTPD.newChunkedResponse(NanoHTTPD.Response.Status.OK,
                "application/json", FileInputStream(f))
        }

        // Added a new handler for uploading files
        manager.register("/config/upload") { session: IHTTPSession ->
            try {
                // Parse the body to handle the multipart form data
                val files = hashMapOf<String, String>()
                session.parseBody(files)

                // Get the file name from the form data
                val fileNames = session.parameters["uploadFile"]
                if (fileNames.isNullOrEmpty()) {
                    return@register NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST,
                        NanoHTTPD.MIME_PLAINTEXT, "No file uploaded.")
                }

                val fileName = fileNames[0]

                // Get the temporary file path where NanoHTTPD stored the uploaded file
                val tempFilePath = files["uploadFile"]
                if (tempFilePath.isNullOrEmpty()) {
                    return@register NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.INTERNAL_ERROR,
                        NanoHTTPD.MIME_PLAINTEXT, "Failed to get temp file path.")
                }

                val uploadedFile = File(tempFilePath)
                val destFile = CONFIG_ROOT.resolve(fileName)

                // Move the temporary file to the final destination
                uploadedFile.renameTo(destFile)

                // Redirect to the config page to see the new file
                val redirectResponse = NanoHTTPD.newFixedLengthResponse(
                    NanoHTTPD.Response.Status.REDIRECT_SEE_OTHER,
                    NanoHTTPD.MIME_HTML,
                    "File uploaded successfully."
                )
                redirectResponse.addHeader("Location", "/config")
                return@register redirectResponse
            } catch (e: Exception) {
                return@register NanoHTTPD.newFixedLengthResponse(
                    NanoHTTPD.Response.Status.INTERNAL_ERROR,
                    NanoHTTPD.MIME_PLAINTEXT,
                    "Upload failed: ${e.message}"
                )
            }
        }
    }
}
