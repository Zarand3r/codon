/**
 * @author Nathan DeVries
 * @date   2019/10/28
 */
#ifndef CURL_FILE_DOWNLOAD_H
#define CURL_FILE_DOWNLOAD_H
#include "src/bullwinkle/all/Handle.h"
#include "src/bullwinkle/all/Signal.h"
#include "src/bullwinkle/all/Slate.h"
#include "src/bullwinkle/all/SlateBuilder.h"
#include "src/bullwinkle/all/core/drone_types.h"
#include "src/bullwinkle/all/core/sxtime.h"
#include "src/bullwinkle/all/hsm/SslPrivateKeyMethod.h"
#include "src/bullwinkle/all/hsm_type_t.enum.h"
#include <curl/curl.h>
namespace Drone
{
    class CurlDispatcher;
    class ExponentialBackoff;
    /**
     * Interface for CurlFileDownlaod, for unit testing.
     */
    class CurlFileDownloadInterface
    {
    public:
        virtual ~CurlFileDownloadInterface() {}
        virtual bool download_file(const std::string &url,
                                   const std::string &connect_to = "",
                                   const int port = -1) = 0;
        virtual bool cancel_download() = 0;
        virtual double get_progress() const = 0;
        virtual std::string get_temp_file_name() const = 0;
        virtual void set_speed_limit(size_t speed_limit) = 0;
        virtual void set_require_ssl(bool required) = 0;
        enum download_status_t
        {
            download_idle,
            download_in_progress,
            download_success,
            download_failed
        };
        virtual download_status_t get_status() const = 0;
    };
    /**
     * Use libcurl to download a file.
     *
     * @note In order to ensure space is available for the file being
     * downloaded, this class reserves space for the maximum possible file size.
     */
    class CurlFileDownload : public SignalHandler,
                             public CurlFileDownloadInterface
    {
    public:
        /**
         * Configuration parameters.
         */
        struct Config
        {
            /**
             * Maximum average download speed, in bytes/second. 0 is unlimited.
             */
            size_t speed_limit = 0;
            /**
             * Maximum file size. Set to -1 to have no limit.
             */
            ssize_t max_file_size = 0;
            /**
             * Timeout, in seconds, for connect phase.
             */
            UINT32 connect_timeout = 60;
            /**
             * Threshold, in bytes/second, below which we'll be considered
             * running at too low a speed.
             */
            UINT32 low_speed_threshold = 10;
            /**
             * Timeout, in seconds, for maximum amount of time we can spend
             * below the low speed threshold before the transfer is considered
             * failed.
             */
            UINT32 low_speed_timeout = 60;
            /**
             * Handler for retry backoff strategy, if not provided, retry after
             * one second.
             */
            Handle<ExponentialBackoff> retry_backoff;
            /**
             * Maximum number of times to retry download. Negative is unlimited.
             */
            INT32 max_retries = 0;
            /**
             * Whether to require SSL.
             */
            bool require_ssl = true;
            /**
             * Path to CA certificate store.
             */
            std::string ca_certs_path;
            /**
             * Path to client certificate.
             */
            std::string client_cert_path;
            /**
             * Path to client key.
             */
            std::string client_key_path;
            /**
             * Path to the cipher store.
             */
            std::string stsafe_cipher_path;
            /**
             * Enable verbose logging.
             */
            bool verbose = false;
            /**
             * Whether to use client auth.
             */
            bool use_client_auth = false;
            /**
             * Whether to use HSM.
             */
            bool use_hsm = false;
            /**
             * The type of HSM used.
             */
            hsm_type_t hsm_type = hsm_type_t::undefined;
            /**
             * The desired permisions for the output file created by
             * CurlFileDownload.
             */
            mode_t output_file_permissions = 0644;
        };
        CurlFileDownload(const std::string &_name);
        virtual ~CurlFileDownload();
        bool init(SlateBuilder &builder, Handle<CurlDispatcher> _curl,
                  const std::string &_temp_file_name, const Config &config,
                  bool lazy_init_temp_file = false);
        bool download_file(const std::string &_url,
                           const std::string &_connect_to = "",
                           const int _port = -1) override;
        bool cancel_download() override;
        double get_progress() const override;
        std::string get_temp_file_name() const override;
        void set_speed_limit(size_t limit) override;
        void set_require_ssl(bool required) override;
        download_status_t get_status() const override;
        nano_t dispatch(nano_t control_time);
        bool move_temp_file_to(const std::string &filename);

    private:
        static size_t write_cb(void *contents, size_t size, size_t nmemb,
                               void *userp);
        static int progress_cb(void *clientp, curl_off_t dltotal,
                               curl_off_t dlnow, curl_off_t ultotal,
                               curl_off_t ulnow);
#ifdef SX_BORINGSSL_ENABLED
        static int modify_ssl_ctx_cb(CURL *easy_handle, void *ctx, void *userp);
#endif
        void update_progress(size_t _dltotal, size_t _dlnow);
        void complete_cb(CURL *easy_handle, CURLcode result);
        void modify_ssl_ctx(void *ctx);
        bool open_temp_file();
        bool setup_and_add_handle();
        /**
         * Name.
         */
        const std::string name;
        /**
         * Slate.
         */
        Slate slate;
        /**
         * Configuration parameters.
         */
        Config config;
        /**
         * Curl dispatcher.
         */
        Handle<CurlDispatcher> curl;
        /**
         * Temporary file name.
         */
        std::string temp_file_name;
        /**
         * File descriptor for our temporary file.
         */
        AutoFd fd;
        /**
         * Curl handle used for tracking this download.
         */
        CURL *handle;
        /**
         * URL of the file we're trying to download.
         */
        std::string url;
        /**
         * Port to connect to for the file we're trying to download.
         * If set to a negative, will just use the curl default.
         */
        int port;
        /**
         * Override hostname we're trying to download from. Empty to use
         * default HTTP Host header.
         *
         * The format should match curl's expectation:
         * HOST:PORT:CONNECT-TO-HOST:CONNECT-TO-PORT
         *
         * Where HOST is the host of the request, PORT is the port of the
         * request, CONNECT-TO-HOST is the host name to connect to (this
         * can also be an ip address), and CONNECT-TO-PORT is the port to
         * connect to.
         */
        std::string connect_to;
        /**
         * Status of the current download.
         */
        WriteToken<INT32> status_tok;
        /**
         * Bytes received in this download.
         */
        WriteToken<UINT64> bytes_received_tok;
        /**
         * Bytes expected in this download.
         */
        WriteToken<UINT64> bytes_expected_tok;
        /**
         * Percent progress in download.
         */
        WriteToken<double> download_progress_tok;
        /**
         * Retry counter.
         */
        WriteToken<UINT32> retry_count_tok;
        /**
         * Whether we're currently waiting to retry.
         */
        WriteToken<bool> pending_retry_tok;
        /**
         * Resume point (bytes).
         */
        WriteToken<UINT32> resume_from_tok;
        /**
         * Last status received from curl.
         */
        WriteToken<INT32> last_curl_status_tok;
        /**
         * Last HTTP status received from curl.
         */
        WriteToken<INT32> last_http_status_tok;
        /**
         * HSM type [hsm_type_t enum]
         */
        WriteToken<INT32> hsm_type_tok;
        /**
         * Error buffer for curl.
         */
        char error_buffer[CURL_ERROR_SIZE];
        /**
         * Linked list of custom connect-to hosts.
         */
        struct curl_slist *connect_to_list;
        /**
         * The SSL_PRIVATE_KEY_METHOD instance.
         */
        Handle<key_method> key_handler;
    };
} /* namespace Drone */
#endif /* CURL_FILE_DOWNLOAD_H */