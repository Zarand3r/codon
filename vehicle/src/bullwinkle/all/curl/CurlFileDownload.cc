/**
 * @author Nathan DeVries
 * @date   2019/10/28
 */
#include "src/bullwinkle/all/curl/CurlFileDownload.h"
#include "src/bullwinkle/all/EventLoop.h"
#include "src/bullwinkle/all/ExponentialBackoff.h"
#include "src/bullwinkle/all/FdEventSink.h"
#include "src/bullwinkle/all/config_file.h"
#include "src/bullwinkle/all/core/sac.h"
#include "src/bullwinkle/all/curl/CurlDispatcher.h"
#include "src/bullwinkle/all/curl/curl_utils.h"
#include "src/bullwinkle/all/hsm/CmrtSslPrivateKeyMethod.h"
#include "src/bullwinkle/all/hsm/SslPrivateKeyMethod.h"
#include "src/bullwinkle/all/hsm/StsafeSslPrivateKeyMethod.h"
#include "src/bullwinkle/all/hsm/TrustZoneSslPrivateKeyMethod.h"
#include <fcntl.h>
#include <unistd.h>
namespace Drone
{
    /**
     * Callback to write files data.
     *
     * @param contents Data to write.
     * @param size Size of each member.
     * @param nmemb Number of members.
     * @param userp User provided pointer (used here for the "this" pointer).
     *
     * @return Number of bytes written.
     */
    size_t CurlFileDownload::write_cb(void *contents, size_t size, size_t nmemb,
                                      void *userp)
    {
        CurlFileDownload *_this = static_cast<CurlFileDownload *>(userp);
        return write(_this->fd.get(), contents, size * nmemb);
    }
    /**
     * Helper callback for progress updates.
     *
     * @see update_progress.
     *
     * @return 0 on success.
     */
    int CurlFileDownload::progress_cb(void *clientp, curl_off_t dltotal,
                                      curl_off_t dlnow,
                                      curl_off_t /* ultotal */,
                                      curl_off_t /* ulnow */)
    {
        CurlFileDownload *_this = static_cast<CurlFileDownload *>(clientp);
        _this->update_progress(dltotal, dlnow);
        return 0;
    }
    /**
     * Callback for transfer progress.
     *
     * @param dltotal Total bytes to we need to download.
     * @param dlnow Total bytes downloaded so far.
     */
    void CurlFileDownload::update_progress(size_t dltotal, size_t dlnow)
    {
        /*
         * If dltotal is zero, we don't actually know the expected size, so
         * leave it as is.
         *
         * We also add "resume_from" into the counters here so these counters
         * reflect the state of the overall transfer. libcurl only reports the
         * progress in the current transfer, and if we resumed from a partially
         * complete transfer, the status from libcurl only includes the
         * remaining data to be transferred.
         */
        if (dltotal != 0)
        {
            slate[bytes_expected_tok] = slate[resume_from_tok] + dltotal;
        }
        slate[bytes_received_tok] = slate[resume_from_tok] + dlnow;
        slate[download_progress_tok] =
            slate[bytes_expected_tok]
                ? static_cast<double>(slate[bytes_received_tok]) /
                      slate[bytes_expected_tok]
                : 0.0;
        /*
         * If we've made any progress, reset the retry backoff.
         */
        if (dlnow > 0 && config.retry_backoff)
        {
            SacIfNot(config.retry_backoff->reset());
        }
    }
#ifdef SX_BORINGSSL_ENABLED
    /**
     * Callback for making modifications to the ssl context.
     *
     * @param easy_handle A pointer to our handle.
     * @param ctx A pointer to the SSL context.
     * @param userp User provided pointer (used here for the "this" pointer).
     *
     * @return 0 on success.
     */
    int CurlFileDownload::modify_ssl_ctx_cb(CURL *, void *ctx, void *userp)
    {
        CurlFileDownload *_this = static_cast<CurlFileDownload *>(userp);
        _this->modify_ssl_ctx(ctx);
        return 0;
    }
#endif
    /**
     * Constructor.
     */
    CurlFileDownload::CurlFileDownload(const std::string &_name)
        : name(_name), slate(), config(), curl(), temp_file_name(), fd(),
          handle(nullptr), url(), port(), status_tok(), bytes_received_tok(),
          bytes_expected_tok(), retry_count_tok(), resume_from_tok(),
          last_curl_status_tok(), last_http_status_tok(), hsm_type_tok(),
          error_buffer(), connect_to_list(nullptr), key_handler()
    {}
    /**
     * Destructor to free manually allocated memory in off-nominal cases.
     */
    CurlFileDownload::~CurlFileDownload()
    {
        if (handle)
        {
            curl->remove_handle(handle);
            curl_easy_cleanup(handle);
            handle = nullptr;
        }
        if (connect_to_list)
        {
            curl_slist_free_all(connect_to_list);
            connect_to_list = nullptr;
        }
    }
    /**
     * Initialize this object.
     *
     * @param builder Slate Builder
     * @param _curl Curl handler.
     * @param _temp_file_name Temporary file name.
     * @param _config Configuration parameters.
     * @param _lazy_init_temp_file Wait until first download to initialize the
     * temp file. (default false). This is needed if storage will not be ready
     * during the init phase.
     *
     * @return True on success.
     */
    bool CurlFileDownload::init(SlateBuilder &builder,
                                Handle<CurlDispatcher> _curl,
                                const std::string &_temp_file_name,
                                const Config &_config,
                                bool _lazy_init_temp_file)
    {
        slate = builder.slate(slate_no_validation);
        SlateBuilder sub_slate = builder.sub_slate(name);
        curl = _curl;
        temp_file_name = _temp_file_name;
        config = _config;
        /*
         * Create our slate tokens.
         */
        SacAbortIfNot(sub_slate.create("status", download_idle, shard_nonsync,
                                       slate_read_only, status_tok),
                      false);
        SacAbortIfNot(sub_slate.create("bytes_received", 0, shard_nonsync,
                                       slate_read_only, bytes_received_tok),
                      false);
        SacAbortIfNot(sub_slate.create("bytes_expected", 0, shard_nonsync,
                                       slate_read_only, bytes_expected_tok),
                      false);
        SacAbortIfNot(sub_slate.create("download_progress", 0, shard_nonsync,
                                       slate_read_only, download_progress_tok),
                      false);
        SacAbortIfNot(sub_slate.create("retry_count", 0, shard_nonsync,
                                       slate_read_only, retry_count_tok),
                      false);
        SacAbortIfNot(sub_slate.create("pending_retry", 0, shard_nonsync,
                                       slate_read_only, pending_retry_tok),
                      false);
        SacAbortIfNot(sub_slate.create("resume_from", 0, shard_nonsync,
                                       slate_read_only, resume_from_tok),
                      false);
        SacAbortIfNot(sub_slate.create("last_curl_status", 0, shard_nonsync,
                                       slate_read_only, last_curl_status_tok),
                      false);
        SacAbortIfNot(sub_slate.create("last_http_status", 0, shard_nonsync,
                                       slate_read_only, last_http_status_tok),
                      false);
        SacAbortIfNot(sub_slate.create("hsm_type", 0, shard_nonsync,
                                       slate_read_only, hsm_type_tok),
                      false);
        /*
         * Make sure we configured a retry backoff if we allow retries.
         */
        SacAbortIf(config.max_retries != 0 && !config.retry_backoff, false);
        /*
         * If we don't have a storage ready token, create the temp file now.
         * If we do, we need to wait until dispatch, so we know if storage is
         * ready.
         */
        if (!_lazy_init_temp_file)
        {
            SacAbortIfNot(open_temp_file(), false);
        }
        return true;
    }
    /**
     * Open the temporary file used for the download. If a max file size is
     * configured, expand the temp file to the maximum size.
     *
     * If output file permissions are configured, set the premissions of the
     * resultant file to the specified permissions. Otherwise, use the default
     * file permissions of 0644.
     *
     * @return True on success.
     */
    bool CurlFileDownload::open_temp_file()
    {
        SacAbortIf(config.max_file_size == 0, false);
        SacAbortOnErrno(fd = open(temp_file_name.c_str(), O_CREAT | O_RDWR,
                                  config.output_file_permissions),
                        false);
        if (config.max_file_size > 0)
        {
            /*
             * Preallocate space in our temp file. This guarantees we'll have
             * enough space to write out the file.
             */
            SacAbortOnErrno(posix_fallocate(fd.get(), 0, config.max_file_size),
                            false);
        }
        return true;
    }
    /**
     * Dispatch.
     *
     * @param control_time Control time.
     *
     * @return Next time to wake up.
     */
    nano_t CurlFileDownload::dispatch(nano_t control_time)
    {
        nano_t next_time = nano_t_max;
        if (slate[pending_retry_tok] &&
            config.retry_backoff->is_due(control_time, next_time))
        {
            if (fd.get() == -1)
            {
                SacIfNot(open_temp_file());
            }
            slate[retry_count_tok]++;
            SacIfNot(setup_and_add_handle());
            slate[pending_retry_tok] = false;
        }
        return next_time;
    }
    /**
     * Download a file.
     *
     * @param url URL to download from.
     * @param connect_to Optionally override the server in the Host header
     * to a custom value. When using a proxy, this should be equal to the
     * hostname of the server that ultimately serves the request. Use an
     * empty string to keep the default server provided by the URL Host
     * header.
     * @param port The port to connect to on the server.
     *             Set to -1 to just use the default.
     *
     * @return True on success.
     */
    bool CurlFileDownload::download_file(const std::string &_url,
                                         const std::string &_connect_to,
                                         const int _port)
    {
        SacAbortIf(slate[status_tok] == download_in_progress, false);
        if (fd.get() == -1)
        {
            SacAbortIfNot(open_temp_file(), false);
        }
        if (config.verbose)
        {
            dbnprintf(200, "Start download of %s...\n", _url.c_str());
        }
        SacAbortOnErrno(lseek(fd.get(), 0, SEEK_SET), false);
        if (config.max_file_size < 0)
        {
            SacAbortOnErrno(::ftruncate(fd.get(), 0), false);
        }
        slate[bytes_received_tok] = 0;
        slate[bytes_expected_tok] = 0;
        url = _url;
        port = _port;
        connect_to = _connect_to;
        if (config.retry_backoff)
        {
            SacIfNot(config.retry_backoff->reset());
        }
        SacAbortIfNot(setup_and_add_handle(), false);
        slate[status_tok] = download_in_progress;
        return true;
    }
    /**
     * Create and set up a libcurl handle, and add that handle to the curl
     * dispatcher.
     *
     * @return True on success.
     */
    bool CurlFileDownload::setup_and_add_handle()
    {
        SacAbortIf(fd.get() == -1, false);
        handle = curl_easy_init();
        SacAbortIf(handle == NULL, false);
        /*
         * Set the URL we intend to retrieve.
         */
        SacAbortOnCurlErr(curl_easy_setopt(handle, CURLOPT_URL, url.c_str()),
                          false);
        /*
         * Set the PORT we intend to retrieve.
         */
        if (port > 0)
        {
            SacAbortOnCurlErr(curl_easy_setopt(handle, CURLOPT_PORT, port),
                              false);
        }
        /*
         * Set up the write callback.
         */
        SacAbortOnCurlErr(
            curl_easy_setopt(handle, CURLOPT_WRITEFUNCTION, &write_cb), false);
        SacAbortOnCurlErr(curl_easy_setopt(handle, CURLOPT_WRITEDATA, this),
                          false);
        /*
         * Get progress updates.
         */
        SacAbortOnCurlErr(
            curl_easy_setopt(handle, CURLOPT_XFERINFOFUNCTION, &progress_cb),
            false);
        SacAbortOnCurlErr(curl_easy_setopt(handle, CURLOPT_XFERINFODATA, this),
                          false);
        SacAbortOnCurlErr(curl_easy_setopt(handle, CURLOPT_NOPROGRESS, 0),
                          false);
        /*
         * Make libcurl tell us about http errors.
         */
        SacAbortOnCurlErr(curl_easy_setopt(handle, CURLOPT_FAILONERROR, 1L),
                          false);
        /*
         * Tell curl to follow redirects automatically.
         */
        SacAbortOnCurlErr(curl_easy_setopt(handle, CURLOPT_FOLLOWLOCATION, 1L),
                          false);
        /*
         * Set the maximum file size.
         */
        if (config.max_file_size > 0)
        {
            SacAbortOnCurlErr(
                curl_easy_setopt(handle, CURLOPT_MAXFILESIZE_LARGE,
                                 static_cast<curl_off_t>(config.max_file_size)),
                false);
        }
        if (config.verbose)
        {
            SacAbortOnCurlErr(curl_easy_setopt(handle, CURLOPT_VERBOSE, 1),
                              false);
        }
        /**
         * Force downloader to use 256 bit encryption. See SATSW-52489.
         */
        SacAbortOnCurlErr(
            curl_easy_setopt(
                handle, CURLOPT_SSL_CIPHER_LIST,
                "ECDHE-ECDSA-AES256-GCM-SHA384:ECDHE-RSA-AES256-GCM-SHA384"),
            false);
        /*
         * Provide a buffer for curl to store errors messages in.
         */
        SacAbortOnCurlErr(
            curl_easy_setopt(handle, CURLOPT_ERRORBUFFER, error_buffer), false);
        /*
         * Set the buffer to empty prior to performing the request.
         */
        error_buffer[0] = 0;
        /*
         * We always allow HTTPS, and allow HTTP if SSL is not required.
         */
        long allowed_protocols = CURLPROTO_HTTPS;
        if (!config.require_ssl)
        {
            allowed_protocols |= CURLPROTO_HTTP;
        }
        SacAbortOnCurlErr(
            curl_easy_setopt(handle, CURLOPT_PROTOCOLS, allowed_protocols),
            false);
        if (config.use_hsm)
        {
#ifdef SX_BORINGSSL_ENABLED
            /*
             * Setup the SSL callback so that the SSL context can be modified.
             */
            SacAbortOnCurlErr(
                curl_easy_setopt(handle, CURLOPT_SSL_CTX_DATA, this), false);
            SacAbortOnCurlErr(curl_easy_setopt(handle, CURLOPT_SSL_CTX_FUNCTION,
                                               modify_ssl_ctx_cb),
                              false);
#endif
        }
        else
        {
            /*
             * Set up CA store for SSL if provided.
             */
            if (!config.ca_certs_path.empty())
            {
                SacAbortOnCurlErr(
                    curl_easy_setopt(handle, CURLOPT_CAINFO,
                                     config.ca_certs_path.c_str()),
                    false);
            }
            /*
             * Set up client cert and key if provided.
             */
            if (!config.client_cert_path.empty())
            {
                SacAbortOnCurlErr(
                    curl_easy_setopt(handle, CURLOPT_SSLCERT,
                                     config.client_cert_path.c_str()),
                    false);
            }
            if (!config.client_key_path.empty())
            {
                SacAbortOnCurlErr(
                    curl_easy_setopt(handle, CURLOPT_SSLKEY,
                                     config.client_key_path.c_str()),
                    false);
            }
        }
        /*
         * Set speed limit.
         */
        SacAbortOnCurlErr(
            curl_easy_setopt(handle, CURLOPT_MAX_RECV_SPEED_LARGE,
                             static_cast<curl_off_t>(config.speed_limit)),
            false);
        /*
         * Fail out of curl if we average too low a rate for too long.  This
         * likely indicates LOS while downloading.
         */
        SacAbortOnCurlErr(curl_easy_setopt(handle, CURLOPT_LOW_SPEED_TIME,
                                           config.low_speed_timeout),
                          false);
        SacAbortOnCurlErr(curl_easy_setopt(handle, CURLOPT_LOW_SPEED_LIMIT,
                                           config.low_speed_threshold),
                          false);
        /*
         * Fail out if we don't connect to the server within the specified
         * timeout.  This likely indicates we're LOS or otherwise don't have
         * connectivity to the server.
         */
        SacAbortOnCurlErr(curl_easy_setopt(handle, CURLOPT_CONNECTTIMEOUT,
                                           config.connect_timeout),
                          false);
        /*
         * If this is a partially complete download, try to resume. Note this
         * assumes the file we are downloading hasn't changed out from under
         * us.
         */
        curl_off_t resume_from = 0;
        SacAbortOnErrno(resume_from = lseek(fd.get(), 0, SEEK_CUR), false);
        slate[resume_from_tok] = resume_from;
        if (resume_from != 0)
        {
            if (config.verbose)
            {
                dbnprintf(100, "Resuming download from %lld\n",
                          static_cast<long long>(resume_from));
            }
            SacAbortOnCurlErr(curl_easy_setopt(handle,
                                               CURLOPT_RESUME_FROM_LARGE,
                                               resume_from),
                              false);
        }
        /*
         * If using a custom connect_to field (generally with a reverse proxy)
         * set that up as a custom option.
         */
        if (!connect_to.empty())
        {
            struct curl_slist *new_list = nullptr;
            new_list = curl_slist_append(connect_to_list, connect_to.c_str());
            if (!new_list)
            {
                SacMsgAbort(false, 100, "curl_slist_append failure.");
            }
            connect_to_list = new_list;
            SacAbortOnCurlErr(
                curl_easy_setopt(handle, CURLOPT_CONNECT_TO, connect_to_list),
                false);
        }
        SacAbortIfNot(
            curl->add_handle(handle,
                             make_slot(*this, &CurlFileDownload::complete_cb)),
            false);
        return true;
    }
    /**
     * Callback for download completion.
     *
     * @param easy_handle A pointer to our handle.
     * @param result The result of the download operation.
     *
     */
    void CurlFileDownload::complete_cb(CURL *easy_handle, CURLcode result)
    {
        if (SacIfNot(easy_handle == handle))
        {
            return;
        }
        long http_response_code = 0;
        curl_easy_getinfo(handle, CURLINFO_RESPONSE_CODE, &http_response_code);
        slate[last_curl_status_tok] = result;
        slate[last_http_status_tok] = http_response_code;
        curl_easy_cleanup(handle);
        handle = nullptr;
        if (connect_to_list)
        {
            curl_slist_free_all(connect_to_list);
            connect_to_list = nullptr;
        }
        /*
         * If we succeeded, set our state to success and return.
         */
        if (result == CURLE_OK)
        {
            /*
             * Sync file to storage.
             */
            SacOnErrno(::fsync(fd.get()));
            if (config.verbose)
            {
                dbnprintf(100, "%s download complete\n", url.c_str());
            }
            slate[status_tok] = download_success;
            return;
        }
        /*
         * Otherwise, figure out the type of failure and decide whether to
         * retry.
         */
        const bool retries_left =
            (config.max_retries < 0 ||
             slate[retry_count_tok] < static_cast<UINT32>(config.max_retries));
        bool can_retry = false;
        /*
         * Check for error codes which indicate connection failures or timeouts.
         */
        if (result == CURLE_OPERATION_TIMEDOUT ||
            result == CURLE_COULDNT_RESOLVE_PROXY ||
            result == CURLE_COULDNT_RESOLVE_HOST ||
            result == CURLE_COULDNT_CONNECT ||
            result == CURLE_SSL_CONNECT_ERROR || result == CURLE_RECV_ERROR)
        {
            can_retry = true;
        }
        /*
         * We got an HTTP error. Check what kind it is to decide if we
         * should retry or fail.
         */
        else if (result == CURLE_HTTP_RETURNED_ERROR)
        {
            /*
             * 500s indicate server errors, so allow these to retry.
             */
            if (config.verbose)
            {
                dbnprintf(100, "HTTP status was: %ld\n", http_response_code);
            }
            if (http_response_code >= 500 && http_response_code < 600)
            {
                can_retry = true;
            }
        }
        else if (result == CURLE_PARTIAL_FILE)
        {
            if (config.verbose)
            {
                dbnprintf(
                    100,
                    "HTTP status was: %ld. Refetching the last five MiB.\n",
                    http_response_code);
            }
            curl_off_t resume_from = 0;
            const off_t FIVE_MIB = 5 * 1024 * 1024;
            off_t move_cursor_back = -1 * FIVE_MIB;
            /*
             * If we haven't received 5 MiB worth of data, erase our progress.
             * Rewind our cursor by how many bytes we have received.
             * We rewind the cursor so that we can recover from receiving
             * any corrupted packets. We chose 5 MiB as a heuristic as it is
             * more than twice the size of any missing chunk of bytes
             * that we have seen. Read more on SATSW-43344.
             */
            if (slate[bytes_received_tok] < static_cast<uint64_t>(FIVE_MIB))
            {
                move_cursor_back = -1 * slate[bytes_received_tok];
            }
            SacOnErrno(resume_from =
                           lseek(fd.get(), move_cursor_back, SEEK_CUR));
            slate[bytes_received_tok] = resume_from;
            can_retry = true;
        }
        if (!can_retry || config.verbose)
        {
            dbnprintf(200, "Error downloading %s: \"%s\"\n", url.c_str(),
                      curl_easy_strerror(result));
            if (strlen(error_buffer) != 0)
            {
                dbnprintf(CURL_ERROR_SIZE + 50,
                          "Additional information \"%s\"\n", error_buffer);
            }
        }
        if (retries_left && can_retry)
        {
            slate[pending_retry_tok] = true;
        }
        else
        {
            slate[status_tok] = download_failed;
        }
    }
    /**
     * Cancel the download.
     */
    bool CurlFileDownload::cancel_download()
    {
        slate[pending_retry_tok] = false;
        if (handle)
        {
            curl->remove_handle(handle);
            curl_easy_cleanup(handle);
            handle = nullptr;
        }
        if (connect_to_list)
        {
            curl_slist_free_all(connect_to_list);
            connect_to_list = nullptr;
        }
        slate[status_tok] = download_idle;
        return true;
    }
    /**
     * Get download status
     *
     * @return Download status.
     */
    CurlFileDownload::download_status_t CurlFileDownload::get_status() const
    {
        return static_cast<download_status_t>(slate[status_tok]);
    }
    /**
     * Get download progress
     *
     * @return Download progress, 0 to 1.
     */
    double CurlFileDownload::get_progress() const
    {
        return slate[download_progress_tok];
    }
    /**
     * Get temporary file name.
     *
     * @return Temporary file name
     */
    std::string CurlFileDownload::get_temp_file_name() const
    {
        return temp_file_name;
    }
    /**
     * Set ssl required.
     *
     * @param required New ssl required value.
     */
    void CurlFileDownload::set_require_ssl(bool required)
    {
        config.require_ssl = required;
    }
    /**
     * Set speed limit.
     *
     * @param limit New speed limit, in bytes per second. 0 is unlimited.
     */
    void CurlFileDownload::set_speed_limit(size_t limit)
    {
        config.speed_limit = limit;
    }
    /**
     * Moves the temporary download to a final location. Only allowed after
     * a download is finished. Recreates the temp file so it's ready for the
     * next download.
     *
     * @param filename Destination location of the file.
     *
     * @return True on success.
     */
    bool CurlFileDownload::move_temp_file_to(const std::string &filename)
    {
        SacAbortIf(slate[status_tok] == download_in_progress, false);
        /*
         * rename() syscall offers atomic replacement of the destination
         * file, so it's the safest choice.
         */
        SacAbortOnErrno(::rename(temp_file_name.c_str(), filename.c_str()),
                        false);
        /*
         * We must reopen the fd otherwise it will point to the new location
         * instead of the temp file location.
         */
        SacAbortIfNot(open_temp_file(), false);
        return true;
    }
#ifdef SX_BORINGSSL_ENABLED
    /**
     * Callback for modifying the SSL context.
     *
     * @param ssl_ctx The SSL context to modify.
     */
    void CurlFileDownload::modify_ssl_ctx(void *ssl_ctx)
    {
        SSL_CTX *ctx = static_cast<SSL_CTX *>(ssl_ctx);
        if (SacIfNot(file_exists(config.client_cert_path.c_str())))
        {
            dbnprintf(100, "Missing file %s.\n",
                      config.client_cert_path.c_str());
            return;
        }
        if (SacIfNot(file_exists(config.ca_certs_path.c_str())))
        {
            dbnprintf(100, "Missing file %s.\n", config.ca_certs_path.c_str());
            return;
        }
        slate[hsm_type_tok] = config.hsm_type;
        switch (config.hsm_type)
        {
        case hsm_type_t::stsafe: {
            std::string stsafe_cipher;
            if (SacIfNot(read_str(config.stsafe_cipher_path, stsafe_cipher)))
            {
                dbnprintf(100, "read_str failed for file %s.\n",
                          config.stsafe_cipher_path.c_str());
                return;
            }
            key_method *p;
            StsafeIdentityConfig stsafe_config;
            SacAssert(Drone::create_external_stsafe_method(stsafe_cipher, p,
                                                            stsafe_config));
            key_handler.assume_ownership(p);
            static uint16_t pref_curve = 0;
            switch (stsafe_config.curve)
            {
            case CurveId::NIST_P_256:
                pref_curve = SSL_SIGN_ECDSA_SECP256R1_SHA256;
                break;
            case CurveId::NIST_P_384:
                pref_curve = SSL_SIGN_ECDSA_SECP384R1_SHA384;
                break;
            default:
                dbnprintf(100, "Invalid curve id %d\n",
                          static_cast<int>(stsafe_config.curve));
                return;
            };
            if (SSL_CTX_set_signing_algorithm_prefs(ctx, &pref_curve, 1) != 1)
            {
                dbnprintf(100, "Unable to set signing algorithm prefs.");
                return;
            }
        }
        break;
        case hsm_type_t::trustzone: {
            std::string wrapped_key;
            if (SacIfNot(read_str_binary(config.client_key_path, wrapped_key)))
            {
                SacPrefix();
                dbnprintf(100, "Failed to read wrapped key file: %s.\n",
                          config.client_key_path.c_str());
                return;
            }
            std::vector<uint8_t> trustzone_wrapped_key(wrapped_key.begin(),
                                                       wrapped_key.end());
            key_method *p;
            SacAssert(Drone::create_external_trustzone_method(
                p, trustzone_wrapped_key));
            key_handler.assume_ownership(p);
            static uint16_t pref_curve = SSL_SIGN_ED25519;
            if (SSL_CTX_set_signing_algorithm_prefs(ctx, &pref_curve, 1) != 1)
            {
                dbnprintf(100, "Unable to set signing algorithm prefs.");
                return;
            }
        }
        break;
        case hsm_type_t::cmrt: {
            key_method *p;
            SacAssert(Drone::create_external_cmrt_method(p));
            key_handler.assume_ownership(p);
            static uint16_t pref_curve = SSL_SIGN_ED25519;
            if (SSL_CTX_set_signing_algorithm_prefs(ctx, &pref_curve, 1) != 1)
            {
                dbnprintf(100, "Unable to set signing algorithm prefs.");
                return;
            }
        }
        break;
        default:
            SacPrefix();
            dbnprintf(
                200, "Expected hsm type to be one of ['stsafe', 'trustzone'].");
            return;
        }
        SSL_CTX_set_private_key_method(ctx, key_handler.get());
        if (SacIfNot(SSL_CTX_use_certificate_file(
                ctx, config.client_cert_path.c_str(), SSL_FILETYPE_PEM)))
        {
            dbnprintf(100, "Unable to read the file %s\n",
                      config.client_cert_path.c_str());
            return;
        }
        if (SacIfNot(SSL_CTX_load_verify_locations(
                ctx, config.ca_certs_path.c_str(), nullptr)))
        {
            dbnprintf(100, "Unable to use the file %s\n",
                      config.ca_certs_path.c_str());
            return;
        }
        SSL_CTX_set_verify(ctx, SSL_VERIFY_PEER, nullptr);
        // This is needed because BoringSSL enabled v1.3 as the max version
        // by default, and the combination of curl and TrustZone doesn't
        // work with v1.3.
        // https://github.com/google/boringssl/commit/58d56f4c59969a23e5f52014e2651c76fea2f877
        SSL_CTX_set_max_proto_version(ctx, TLS1_2_VERSION);
    }
#endif
} // namespace Drone
/* namespace Drone */